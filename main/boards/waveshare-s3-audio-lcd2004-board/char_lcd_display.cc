#include "char_lcd_display.h"
#include <algorithm>
#include <cstring>
#include <string>
#include <vector>
#include <esp_log.h>
#include "application.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <ctime>

// -----------------------------------------------------------------------------
// Global handle used by LCD I2C write callback
// -----------------------------------------------------------------------------
static volatile uint32_t g_message_token = 0;

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

    lcd_mutex_ = xSemaphoreCreateMutex();

}

CharLcdDisplay::~CharLcdDisplay()
{
    if (lcd_mutex_) {
        vSemaphoreDelete(lcd_mutex_);
        lcd_mutex_ = nullptr;
    }
}

void CharLcdDisplay::SetChatMessage(const char * /*role*/, const char *content)
{
    if (!content) { if (Lock(50)) { lcd_clear(); Unlock(); } return; }
    std::string filtered = lcd_filter_text(content);
    for (size_t k = 0; k < filtered.size(); ++k) {
        filtered[k] = hd44780_sanitize_char(filtered[k]);
    }

    std::vector<std::string> lines;
    lines.reserve((filtered.size() + cols_ - 1) / cols_);
    for (size_t off = 0; off < filtered.size(); off += cols_) {
        lines.emplace_back(filtered.substr(off, std::min(static_cast<size_t>(cols_), filtered.size() - off)));
    }

    if (!Lock(200)) { return; }
    lcd_clear();
    vTaskDelay(pdMS_TO_TICKS(2));
    int delay_ms = 20;
    int initial_rows = std::min(rows_, static_cast<int>(lines.size()));
    uint32_t my_token = g_message_token + 1;
    g_message_token = my_token;
    for (int r = 0; r < initial_rows; ++r) {
        lcd_set_cursor(0, r);
        vTaskDelay(pdMS_TO_TICKS(1));
        const auto &ln = lines[r];
        for (size_t i = 0; i < ln.size(); ++i) {
            char s[2] = { hd44780_sanitize_char(ln[i]), 0 };
            lcd_write_string(s);
            vTaskDelay(pdMS_TO_TICKS(delay_ms));
        }
        for (int c = static_cast<int>(ln.size()); c < cols_; ++c) {
            char s[2] = { ' ', 0 };
            lcd_write_string(s);
        }
        //vTaskDelay(pdMS_TO_TICKS(delay_ms));
        if (my_token != g_message_token) { Unlock(); return; }
    }
    for (int r = initial_rows; r < rows_; ++r) {
        lcd_set_cursor(0, r);
        vTaskDelay(pdMS_TO_TICKS(1));
        for (int c = 0; c < cols_; ++c) {
            char s[2] = { ' ', 0 };
            lcd_write_string(s);
        }
        //vTaskDelay(pdMS_TO_TICKS(delay_ms));
        if (my_token != g_message_token) { Unlock(); return; }
    }

for (size_t i = rows_; i < lines.size(); ++i) {
    size_t start = i - rows_ + 1;

    // 1. Draw the “scrolled up” existing lines quickly (no delay)
    for (int r = 0; r < rows_ - 1; ++r) {
        const auto &ln = lines[start + r];
        lcd_set_cursor(0, r);
        vTaskDelay(pdMS_TO_TICKS(1));
        for (size_t k = 0; k < ln.size(); ++k) {
            char s[2] = { ln[k], 0 };
            lcd_write_string(s);
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        for (int c = (int)ln.size(); c < cols_; ++c)
        {
            char s[2] = { ' ', 0 };
            lcd_write_string(s);
        }
        if (my_token != g_message_token) { Unlock(); return; }
    }

    const auto &ln_new = lines[start + rows_ - 1];
    lcd_set_cursor(0, rows_ - 1);
    vTaskDelay(pdMS_TO_TICKS(1));
    for (size_t k = 0; k < ln_new.size(); ++k) {
        char s[2] = { ln_new[k], 0 };
        lcd_write_string(s);
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
        if (my_token != g_message_token) { Unlock(); return; }
    }
    for (int c = (int)ln_new.size(); c < cols_; ++c)
    {
        char s[2] = { ' ', 0 };
        lcd_write_string(s);
    }
    if (my_token != g_message_token) { Unlock(); return; }
    }
    Unlock();
}

void CharLcdDisplay::SetStatus(const char* status)
{
    std::string filtered = lcd_filter_text(status ? status : "");
    for (size_t k = 0; k < filtered.size(); ++k) filtered[k] = hd44780_sanitize_char(filtered[k]);
    if (!Lock(100)) return;
    g_message_token++;
    lcd_set_cursor(0, 0);
    size_t n = std::min<size_t>(filtered.size(), cols_);
    for (size_t i = 0; i < n; ++i) { char s[2] = { filtered[i], 0 }; lcd_write_string(s); }
    for (int c = (int)n; c < cols_; ++c) { char s[2] = { ' ', 0 }; lcd_write_string(s); }
    Unlock();
}

void CharLcdDisplay::ShowNotification(const std::string& notification, int duration_ms)
{
    ShowNotification(notification.c_str(), duration_ms);
}

void CharLcdDisplay::ShowNotification(const char* notification, int /*duration_ms*/)
{
    std::string filtered = lcd_filter_text(notification ? notification : "");
    for (size_t k = 0; k < filtered.size(); ++k) filtered[k] = hd44780_sanitize_char(filtered[k]);
    if (!Lock(100)) return;
    g_message_token++;
    lcd_set_cursor(0, 1);
    size_t n = std::min<size_t>(filtered.size(), cols_);
    for (size_t i = 0; i < n; ++i) { char s[2] = { filtered[i], 0 }; lcd_write_string(s); }
    for (int c = (int)n; c < cols_; ++c) { char s[2] = { ' ', 0 }; lcd_write_string(s); }
    Unlock();
}

void CharLcdDisplay::SetEmotion(const char* /*emotion*/)
{
}

void CharLcdDisplay::SetPowerSaveMode(bool on)
{
    if (Lock(10)) { lcd_backlight(!on); Unlock(); }
}

void CharLcdDisplay::UpdateStatusBar(bool /*update_all*/)
{
    auto& app = Application::GetInstance();
    auto curr = app.GetDeviceState();
    bool on = !(curr == kDeviceStateIdle);
    if (Lock(10)) { lcd_backlight(on); Unlock(); }
    if (curr == kDeviceStateIdle) {
        time_t now = time(NULL);
        struct tm* t = localtime(&now);
        char buf[6];
        if (t) { snprintf(buf, sizeof(buf), "%02d:%02d", t->tm_hour, t->tm_min); }
        else { snprintf(buf, sizeof(buf), "--:--"); }
        if (!Lock(20)) return;
        lcd_set_cursor(cols_ - 5, rows_ - 1);
        for (size_t i = 0; i < 5; ++i) { char s[2] = { buf[i], 0 }; lcd_write_string(s); }
        Unlock();
    }
}
