#include "char_lcd_display.h"

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
    if (!content) { SendClear(); return; }
    std::string filtered = lcd_filter_all(content);

    SendClear();
    SendShow(filtered.c_str(), 0, 0);
}

void CharLcdDisplay::SetStatus(const char* status)
{
    if (status && std::strcmp(status, Lang::Strings::LISTENING) == 0) {
        SendAnimation("listening", -1, -1);
        return;
    }
    std::string filtered = lcd_filter_all(status ? status : "");
    SendShow(filtered.c_str(), 0, 0);
}

void CharLcdDisplay::ShowNotification(const std::string& notification, int duration_ms)
{
    ShowNotification(notification.c_str(), duration_ms);
}

void CharLcdDisplay::ShowNotification(const char* notification, int /*duration_ms*/)
{
    std::string filtered = lcd_filter_all(notification ? notification : "");
    SendShow(filtered.c_str(), 1, 0);
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
    auto curr = app.GetDeviceState();
    
    if (curr == kDeviceStateIdle) {
        time_t now = time(NULL);
        struct tm* t = localtime(&now);
        char buf[12];
        if (t) { snprintf(buf, sizeof(buf), "%02d:%02d", t->tm_hour, t->tm_min); }
        else { snprintf(buf, sizeof(buf), "--:--"); }
        SendShow(buf, rows_ - 1, cols_ - 5);
        if (sensor_valid) { 
            snprintf(buf, sizeof(buf), "%4.1fC %4.1f%%", temp, hum); 
            SendShow(buf, rows_ - 1, 0);
        }
    }    
}

void CharLcdDisplay::SendClear() {
    if (!display_queue_) return;
    DisplayMsg msg{};
    msg.text[0] = '\0';
    msg.row = -100;
    msg.col = -100;
    xQueueSend(display_queue_, &msg, 0);
}

void CharLcdDisplay::SendSetCursor(int row, int col) {
    if (!display_queue_) return;
    DisplayMsg msg{};
    msg.text[0] = '\0';
    msg.row = row;
    msg.col = col;
    xQueueSend(display_queue_, &msg, 0);
}

void CharLcdDisplay::SendShow(const char* text, int row, int col) {
    if (!display_queue_ || !text) return;
    DisplayMsg msg{};
    msg.row = row;
    msg.col = col;
    std::string filtered = lcd_filter_all(text);
    size_t n = std::min<size_t>(sizeof(msg.text) - 1, filtered.size());
    std::memcpy(msg.text, filtered.data(), n);
    msg.text[n] = '\0';
    xQueueSend(display_queue_, &msg, 0);
}

void CharLcdDisplay::SendAnimation(const char* name, int row, int col) {
    if (!display_queue_ || !name) return;
    DisplayMsg msg{};
    msg.row = row;
    msg.col = col;
    size_t n = std::min<size_t>(sizeof(msg.text) - 1, std::strlen(name));
    std::memcpy(msg.text, name, n);
    msg.text[n] = '\0';
    xQueueSend(display_queue_, &msg, 0);
}

void CharLcdDisplay::DisplayTask(void* arg) {
    CharLcdDisplay* self = static_cast<CharLcdDisplay*>(arg);
    DisplayMsg msg;
    for (;;) {
        if (xQueueReceive(self->display_queue_, &msg, portMAX_DELAY) == pdPASS) {
            if (msg.row == -100 && msg.col == -100) {
                lcd_clear();
                self->cursor_row_ = 0;
                self->cursor_col_ = 0;
                continue;
            }
            if (msg.text[0] == '\0') {
                if (msg.row >= 0 && msg.row < self->rows_ && msg.col >= 0 && msg.col < self->cols_) {
                    self->cursor_row_ = msg.row;
                    self->cursor_col_ = msg.col;
                    lcd_set_cursor((uint8_t)self->cursor_col_, (uint8_t)self->cursor_row_);
                }
                continue;
            }
            if (std::strcmp(msg.text, "listening") == 0) {
                int r = (msg.row >= 0 && msg.row < self->rows_) ? msg.row : self->cursor_row_;
                int c = (msg.col >= 0 && msg.col < self->cols_) ? msg.col : self->cursor_col_;
                const uint8_t seq[6] = {1,2,3,4,3,2}; //0 somehow not working
                for (int l = 0; l < 20; ++l) { //max 20 loops, 40 sec.
                    for (int f = 0; f < 6; ++f) {
                        lcd_set_cursor((uint8_t)c, (uint8_t)r);
                        char s[2] = { (char)seq[f], 0 };
                        lcd_write_string(s);
                        // wait 300ms each frame but in chunks so more responsive
                        for (int delay_chunk = 0; delay_chunk < 6; ++delay_chunk) {
                            vTaskDelay(pdMS_TO_TICKS(50));
                            if (uxQueueMessagesWaiting(self->display_queue_) > 0) goto end_animation;
                        }
                    }
                }
                end_animation:   
                continue;
            }
            if (msg.row >= 0 && msg.row < self->rows_ && msg.col >= 0 && msg.col < self->cols_) {
                self->cursor_row_ = msg.row;
                self->cursor_col_ = msg.col;
            }
            if (self->cursor_row_ == 0 && self->cursor_col_ == 0) {
                std::vector<std::string> lines;
                size_t total_len = std::strlen(msg.text);
                for (size_t off = 0; off < total_len && lines.size() < 8; off += self->cols_) {
                    size_t chunk = std::min<size_t>(self->cols_, total_len - off);
                    lines.emplace_back(std::string(msg.text + off, chunk));
                }
                size_t line_count = lines.size();
                size_t page1_lines = std::min<size_t>(self->rows_, line_count);
                for (size_t r = 0; r < page1_lines; ++r) {
                    self->cursor_row_ = (int)r;
                    self->cursor_col_ = 0;
                    for (size_t i = 0; i < lines[r].size(); ++i) {
                        lcd_set_cursor((uint8_t)self->cursor_col_, (uint8_t)self->cursor_row_);
                        char s[2] = { lines[r][i], 0 };
                        lcd_write_string(s);
                        self->cursor_col_++;
                    }
                }
                if (line_count > self->rows_) {
                    vTaskDelay(pdMS_TO_TICKS(500));
                    lcd_clear();
                    self->cursor_row_ = 0;
                    self->cursor_col_ = 0;
                    size_t start2 = line_count > self->rows_ ? (line_count - self->rows_) : 0;
                    for (size_t r = 0; r < self->rows_; ++r) {
                        const std::string& ln = lines[start2 + r];
                        self->cursor_row_ = (int)r;
                        self->cursor_col_ = 0;
                        for (size_t i = 0; i < ln.size(); ++i) {
                            lcd_set_cursor((uint8_t)self->cursor_col_, (uint8_t)self->cursor_row_);
                            char s[2] = { ln[i], 0 };
                            lcd_write_string(s);
                            self->cursor_col_++;
                        }
                    }
                }
            } else {
                const char* text = msg.text;
                int count = 0;
                int max_chars = self->cols_ * self->rows_;
                while (*text && count < max_chars) {
                    if (self->cursor_row_ >= self->rows_) break;
                    if (self->cursor_col_ >= self->cols_) {
                        self->cursor_col_ = 0;
                        self->cursor_row_++;
                        if (self->cursor_row_ >= self->rows_) break;
                    }
                    lcd_set_cursor((uint8_t)self->cursor_col_, (uint8_t)self->cursor_row_);
                    char s[2] = { *text, 0 };
                    lcd_write_string(s);
                    self->cursor_col_++;
                    text++;
                    count++;
                }
            }
        }
    }
}
