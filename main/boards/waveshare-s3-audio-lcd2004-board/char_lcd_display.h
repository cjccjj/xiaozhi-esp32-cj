#ifndef CHAR_LCD_DISPLAY_H
#define CHAR_LCD_DISPLAY_H

#include "display.h"
#include "i2c_lcd.h"
#include <string>
#include <driver/gpio.h>
#include <driver/i2c_master.h>
#include "freertos/semphr.h"

class CharLcdDisplay : public Display {
public:
    CharLcdDisplay(i2c_port_num_t i2c_port,
                   gpio_num_t sda_gpio,
                   gpio_num_t scl_gpio,
                   uint8_t i2c_addr = 0x27,
                   int cols = 20,
                   int rows = 4);

    virtual ~CharLcdDisplay();

    virtual void SetStatus(const char* status) override;
    virtual void ShowNotification(const char* notification, int duration_ms = 3000) override;
    virtual void ShowNotification(const std::string& notification, int duration_ms = 3000) override;
    virtual void SetEmotion(const char* emotion) override;
    virtual void SetChatMessage(const char* role, const char* content) override;
    virtual void UpdateStatusBar(bool update_all = false) override {}
    virtual void SetPowerSaveMode(bool on) override;

protected:
    virtual bool Lock(int timeout_ms = 0) override {
        if (!lcd_mutex_) return true;
        TickType_t ticks = pdMS_TO_TICKS(timeout_ms);
        return xSemaphoreTake(lcd_mutex_, ticks) == pdTRUE;
    }
    virtual void Unlock() override {
        if (lcd_mutex_) xSemaphoreGive(lcd_mutex_);
    }

private:
    i2c_port_num_t i2c_port_;
    gpio_num_t sda_gpio_;
    gpio_num_t scl_gpio_;
    uint8_t i2c_addr_;
    int cols_;
    int rows_;

    SemaphoreHandle_t lcd_mutex_{nullptr};
};

#endif // CHAR_LCD_DISPLAY_H
