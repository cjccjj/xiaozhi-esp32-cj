#include "char_lcd_display.h"
#include <algorithm>
#include <cstring>
#include <string>
#include <esp_log.h>
#include "application.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <ctime>


static const char *TAG = "CharLcdDisplay";

static inline bool hd44780_is_supported(char ch) {
    return ch >= 32 && ch <= 127;
}

static inline char hd44780_sanitize_char(char ch) {
    if (!hd44780_is_supported(ch)) return ' ';
    switch (ch) {
        case '\\':
            return '|';
        case '~':
            return ' ';
        default:
            return ch;
    }
}

static std::string lcd_filter_text(const char* content) {
    if (content == nullptr) return std::string();
    const unsigned char* s = reinterpret_cast<const unsigned char*>(content);
    size_t n = std::strlen(content);
    std::string out;
    out.reserve(n);
    size_t i = 0;
    while (i < n) {
        unsigned char b0 = s[i];
        if (b0 < 0x80) {
            char ch;
            if (b0 == '\n' || b0 == '\r' || b0 == '\t') {
                ch = ' ';
            } else if (b0 < 32 || b0 > 127) {
                ch = ' ';
            } else {
                ch = hd44780_sanitize_char(static_cast<char>(b0));
            }
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
                out.push_back(hd44780_sanitize_char(ch));
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
    std::string filtered = lcd_filter_text(content);
    for (size_t k = 0; k < filtered.size(); ++k) {
        filtered[k] = hd44780_sanitize_char(filtered[k]);
    }

    SendClear();
    SendShow(filtered.c_str(), 0, 0);
}

void CharLcdDisplay::SetStatus(const char* status)
{
    std::string filtered = lcd_filter_text(status ? status : "");
    for (size_t k = 0; k < filtered.size(); ++k) filtered[k] = hd44780_sanitize_char(filtered[k]);
    SendShow(filtered.c_str(), 0, 0);
}

void CharLcdDisplay::ShowNotification(const std::string& notification, int duration_ms)
{
    ShowNotification(notification.c_str(), duration_ms);
}

void CharLcdDisplay::ShowNotification(const char* notification, int /*duration_ms*/)
{
    std::string filtered = lcd_filter_text(notification ? notification : "");
    for (size_t k = 0; k < filtered.size(); ++k) filtered[k] = hd44780_sanitize_char(filtered[k]);
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
        char buf[6];
        if (t) { snprintf(buf, sizeof(buf), "%02d:%02d", t->tm_hour, t->tm_min); }
        else { snprintf(buf, sizeof(buf), "--:--"); }
        SendShow(buf, rows_ - 1, cols_ - 5);
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
    size_t n = std::min<size_t>(sizeof(msg.text) - 1, std::strlen(text));
    for (size_t i = 0; i < n; ++i) {
        msg.text[i] = hd44780_sanitize_char(text[i]);
    }
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
            if (msg.row >= 0 && msg.row < self->rows_ && msg.col >= 0 && msg.col < self->cols_) {
                self->cursor_row_ = msg.row;
                self->cursor_col_ = msg.col;
            }
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
