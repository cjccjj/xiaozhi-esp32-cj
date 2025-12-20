#include "char_lcd_display.h"
#include "zhtopy.h"

#include <algorithm>
#include <cstring>
#include <string>
#include <vector>
#include <esp_log.h>
#include "application.h"
#include "assets/lang_config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <ctime>



static const char *TAG = "CharLcdDisplay";

static std::string lcd_filter_all(const char* content) {
    if (content == nullptr) return std::string();
    const unsigned char* s = reinterpret_cast<const unsigned char*>(content);
    size_t n = std::strlen(content);
    std::string out;
    out.reserve(n);
    size_t i = 0;
    while (i < n) {
        unsigned char b0 = s[i];
        if (b0 < 0x80) {
            char ch = static_cast<char>(b0);
            if (b0 == '\n' || b0 == '\r' || b0 == '\t') ch = ' ';
            if (b0 > 127) ch = ' ';
            if (ch == '\\') ch = '|';
            if (ch == '~') ch = ' ';
            out.push_back(ch);
            i++;
        } else if (b0 >= 0xE0 && b0 <= 0xEF && i + 2 < n) {
            unsigned char b1 = s[i + 1];
            unsigned char b2 = s[i + 2];
            if ((b1 & 0xC0) == 0x80 && (b2 & 0xC0) == 0x80) {
                uint32_t cp = ((b0 & 0x0F) << 12) | ((b1 & 0x3F) << 6) | (b2 & 0x3F);

                // Check if it's a Chinese character (CJK Unified Ideographs)
                if (cp >= 0x4E00 && cp <= 0x9FA5) {
                    // Extract the 3-byte UTF-8 sequence
                    std::string chinese_char(&content[i], 3);
                    
                    // Convert to pinyin and append
                    std::string pinyin = ZhToPY::instance().zhToPY(chinese_char);
                    if (!pinyin.empty()) {
                        out.append(pinyin);
                    } else {
                        out.push_back(' ');  // Fallback for unmapped chars
                    }
                } 

                char ch = ' ';
                if (cp == 0x2018 || cp == 0x2019) ch = '\'';
                else if (cp == 0x201C || cp == 0x201D) ch = '"';
                else if (cp == 0x2013 || cp == 0x2014) ch = '-';
                else if (cp == 0x2026) ch = '_';
                if (ch == '\\') ch = '|';
                if (ch == '~') ch = ' ';
                out.push_back(ch);
                i += 3;
            } else {
                out.push_back(' ');
                i++;
            }
        } else if (b0 >= 0xC2 && b0 <= 0xDF && i + 1 < n) {
            unsigned char b1 = s[i + 1];
            if ((b1 & 0xC0) == 0x80) {
                out.push_back(' ');
                i += 2;
            } else {
                out.push_back(' ');
                i++;
            }
        } else if (b0 >= 0xF0 && b0 <= 0xF4 && i + 3 < n) {
            out.push_back(' ');
            i += 4;
        } else {
            out.push_back(' ');
            i++;
        }
    }
    return out;
}

// -----------------------------------------------------------------------------
CharLcdDisplay::CharLcdDisplay(i2c_port_num_t i2c_port,
                               gpio_num_t sda_gpio,
                               gpio_num_t scl_gpio,
                               uint8_t i2c_addr,
                               int cols,
                               int rows)
    : i2c_port_(i2c_port),
      sda_gpio_(sda_gpio),
      scl_gpio_(scl_gpio),
      i2c_addr_(i2c_addr),
      cols_(cols),
      rows_(rows)
{
    ESP_LOGI(TAG, "Initializing I2C LCD");
    i2c_master_init();
    lcd_init();
    lcd_backlight(true);
    lcd_clear();
    vTaskDelay(pdMS_TO_TICKS(2));

    ESP_LOGI(TAG, "LCD initialized successfully");
    
    display_queue_ = xQueueCreate(10, sizeof(DisplayMsg));
    xTaskCreate(&CharLcdDisplay::DisplayTask, "charlcd_display", 3072, this, 5, &display_task_handle_);
}

CharLcdDisplay::~CharLcdDisplay()
{
    if (display_task_handle_) {
        vTaskDelete(display_task_handle_);
        display_task_handle_ = nullptr;
    }
    if (display_queue_) {
        vQueueDelete(display_queue_);
        display_queue_ = nullptr;
    }
}

void CharLcdDisplay::SetChatMessage(const char * /*role*/, const char *content)
{
    if (!content) { return; }
    //std::string filtered = lcd_filter_all(content);

    SendClear();
    SendShow(content, 0, 0);
}

void CharLcdDisplay::SetStatus(const char* status)
{
    if (status && std::strcmp(status, Lang::Strings::LISTENING) == 0) {
        SendAnimation("listening", -1, -1);
        return;
    }
    SendShow(status, 0, 0);
}

void CharLcdDisplay::ShowNotification(const std::string& notification, int duration_ms)
{
    ShowNotification(notification.c_str(), duration_ms);
}

void CharLcdDisplay::ShowNotification(const char* notification, int /*duration_ms*/)
{
    SendShow(notification, 1, 0);
}

void CharLcdDisplay::SetEmotion(const char* /*emotion*/)
{
}

void CharLcdDisplay::SetPowerSaveMode(bool on)
{
    (void)on;
}

void CharLcdDisplay::UpdateStatusBar(bool /*update_all*/)
{
    auto& app = Application::GetInstance();
    if (app.GetDeviceState() != kDeviceStateIdle) return;

    // 1. Default state: 15 spaces + placeholder time
    char buf[21] = "               --:--";
    time_t now = time(NULL);
    struct tm* t = localtime(&now);

    // 2. Write sensors first (at index 0)
    // The null terminator falls at index 11, which the next snprintf will overwrite
    if (sensor_valid) {
        snprintf(buf, sizeof(buf), "%4.1f\005 %4.1f\006    --:--", temp, hum);
    }

    // 3. Write time (at index 15)
    // This overwrites any previous null terminators and sets the final \0 at index 20
    if (t) {
        snprintf(buf + 15, 6, "%02d:%02d", t->tm_hour, t->tm_min);
    }

    SendShow(buf, rows_ - 1, 0);
    SendSetCursor(0, 0);
}

void CharLcdDisplay::SendClear() {
    if (!display_queue_) return;
    DisplayMsg msg{};
    msg.cmd = LCD_CMD_CLEAR;
    xQueueSend(display_queue_, &msg, 0);
}

void CharLcdDisplay::SendSetCursor(int row, int col) {
    if (!display_queue_) return;
    DisplayMsg msg{};
    msg.cmd = LCD_CMD_SET_CURSOR;
    msg.row = row;
    msg.col = col;
    msg.text[0] = '\0';
    xQueueSend(display_queue_, &msg, 0);
}

void CharLcdDisplay::SendShow(const char* text, int row, int col) {
    if (!display_queue_ || !text) return;
    DisplayMsg msg{};
    msg.cmd = LCD_CMD_SHOW;
    msg.row = row;
    msg.col = col;

    // Filter text (removes non-ASCII/special chars) before sending
    std::string filtered = lcd_filter_all(text);
    size_t n = std::min<size_t>(sizeof(msg.text) - 1, filtered.size());
    std::memcpy(msg.text, filtered.data(), n);
    msg.text[n] = '\0';

    xQueueSend(display_queue_, &msg, 0);
}

void CharLcdDisplay::SendAnimation(const char* name, int row, int col) {
    if (!display_queue_ || !name) return;
    DisplayMsg msg{};
    msg.cmd = LCD_CMD_ANIMATION;
    msg.row = row;
    msg.col = col;

    // Copy the animation name (e.g., "listening") into the message
    size_t n = std::min<size_t>(sizeof(msg.text) - 1, std::strlen(name));
    std::memcpy(msg.text, name, n);
    msg.text[n] = '\0';

    xQueueSend(display_queue_, &msg, 0);
}

void CharLcdDisplay::Lcd_Animation(const DisplayMsg& msg) {
    // Determine row and column (use provided values or current cursor position)
    int r = (msg.row >= 0 && msg.row < rows_) ? msg.row : cursor_row_;
    int c = (msg.col >= 0 && msg.col < cols_) ? msg.col : cursor_col_;

    // 1. Determine which animation was requested
    if (std::strcmp(msg.text, "listening") == 0) {

        // Your animation sequence of custom char no. 
        const uint8_t seq[6] = {1, 2, 3, 4, 3, 2};

        // Max 20 loops (~36-40 seconds total)
        for (int l = 0; l < 20; ++l) { 
            for (int f = 0; f < 6; ++f) {    
                // 1. Move cursor and draw the frame
                lcd_set_cursor((uint8_t)c, (uint8_t)r);
                char s[2] = { (char)seq[f], 0 };
                lcd_write_string(s);

                // 2. Chunked delay: Wait 300ms total (6 * 50ms)
                // This makes the animation "quit" instantly if a new message arrives
                for (int delay_chunk = 0; delay_chunk < 6; ++delay_chunk) {
                    vTaskDelay(pdMS_TO_TICKS(50));
                    
                    // If a new message is waiting in the queue, QUIT current animation
                    if (uxQueueMessagesWaiting(display_queue_) > 0) {
                        return; // Replaces 'goto end_animation'
                    }
                }
            }
        }
    }
    //else if for other animation
}

void CharLcdDisplay::Lcd_Show(const DisplayMsg& msg) {
    // 1. Update cursor position if valid coordinates are provided in the message
    if (msg.row >= 0 && msg.row < rows_ && msg.col >= 0 && msg.col < cols_) {
        cursor_row_ = msg.row;
        cursor_col_ = msg.col;
    }

    // --- LOGIC A: Multi-line Paging (Starts at 0,0) ---
    if (cursor_row_ == 0 && cursor_col_ == 0) {
        std::vector<std::string> lines;
        size_t total_len = std::strlen(msg.text);
        size_t current_pos = 0;

        // Split text into lines, omitting leading spaces on new lines (Max 8 lines as per your original)
        while (current_pos < total_len && lines.size() < 8) {
            size_t start_of_line = current_pos;

            // For lines after the first one, if it starts with a space, skip it.
            if (lines.size() > 0 && msg.text[start_of_line] == ' ') {
                start_of_line++;
            }

            size_t chars_remaining = total_len - start_of_line;
            size_t chunk_len = std::min<size_t>((size_t)cols_, chars_remaining);

            if (chunk_len > 0) {
                lines.emplace_back(std::string(msg.text + start_of_line, chunk_len));
            }

            // Advance the position by the number of characters processed for this line.
            current_pos = start_of_line + chunk_len;
        }

        size_t line_count = lines.size();
        size_t page1_lines = std::min<size_t>((size_t)rows_, line_count);

        // Print Page 1
        for (size_t r = 0; r < page1_lines; ++r) {
            // CHECK: Quit between lines if a new message arrives
            if (uxQueueMessagesWaiting(display_queue_) > 0) return;

            cursor_row_ = (int)r;
            cursor_col_ = 0;
            lcd_set_cursor(0, (uint8_t)cursor_row_);
            lcd_write_string(lines[r].c_str());
            cursor_col_ = lines[r].size();
        }

        // Handle Page 2 transition if text is longer than the screen
        if (line_count > (size_t)rows_) {
            // Wait 500ms total, checking every 50ms for responsiveness
            for (int i = 0; i < 10; ++i) {
                vTaskDelay(pdMS_TO_TICKS(50));
                if (uxQueueMessagesWaiting(display_queue_) > 0) return;
            }

            lcd_clear();
            cursor_row_ = 0;
            cursor_col_ = 0;

            size_t start2 = (line_count > (size_t)rows_) ? (line_count - (size_t)rows_) : 0;
            //drop any char after page2
            for (size_t r = 0; r < (size_t)rows_ && (start2 + r) < line_count; ++r) {
                // CHECK: Quit between lines
                if (uxQueueMessagesWaiting(display_queue_) > 0) return;

                cursor_row_ = (int)r;
                cursor_col_ = 0;
                lcd_set_cursor(0, (uint8_t)cursor_row_);
                lcd_write_string(lines[start2 + r].c_str());
                cursor_col_ = lines[start2 + r].size();
            }
        }
    }
    // --- LOGIC B: Direct Placement (Starts elsewhere) ---
    else {
        const char* text = msg.text;
        int count = 0;
        //drop char when page end.
        int max_chars = (rows_ - cursor_row_) * cols_ - cursor_col_;
        bool is_first_char_of_message = true;

        while (*text && count < max_chars) {
            if (cursor_row_ >= rows_) break;

            // NEW: If we are at the start of a new line (due to wrapping) and the character is a space, skip it.
            if (!is_first_char_of_message && cursor_col_ == 0 && *text == ' ') {
                text++; // Skip the leading space
                if (!*text) continue; // If that space was the last character, exit.
            }

            // 1. Draw the character
            lcd_set_cursor((uint8_t)cursor_col_, (uint8_t)cursor_row_);
            char s[2] = { *text, 0 };
            lcd_write_string(s);
            is_first_char_of_message = false;

            // 2. Advance positions
            cursor_col_++;
            text++;
            count++;

            // 3. Handle line wrap AND Check for new messages
            if (cursor_col_ >= cols_) {
                cursor_col_ = 0;
                cursor_row_++;

                // AFTER a line is finished, we check if we should quit for a new job
                if (uxQueueMessagesWaiting(display_queue_) > 0) return;
            }
        }
    }
}

void CharLcdDisplay::DisplayTask(void* arg) {
    CharLcdDisplay* self = static_cast<CharLcdDisplay*>(arg);
    DisplayMsg msg;
    for (;;) {
        if (xQueueReceive(self->display_queue_, &msg, portMAX_DELAY) == pdPASS) {
            switch (msg.cmd) {
            case LCD_CMD_CLEAR:
                lcd_clear();
                self->cursor_row_ = 0;
                self->cursor_col_ = 0;
                break;
            case LCD_CMD_SET_CURSOR:
                if (msg.row >= 0 && msg.row < self->rows_ && msg.col >= 0 && msg.col < self->cols_) {
                    self->cursor_row_ = msg.row;
                    self->cursor_col_ = msg.col;
                    lcd_set_cursor((uint8_t)self->cursor_col_, (uint8_t)self->cursor_row_);
                }
                break;
            case LCD_CMD_ANIMATION:
                self->Lcd_Animation(msg);
                break;
            case LCD_CMD_SHOW:
                self->Lcd_Show(msg);
                    break;
            }
        }
    }
}
