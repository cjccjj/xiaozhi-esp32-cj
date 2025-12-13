#ifndef CHAR_LCD_DISPLAY_H
#define CHAR_LCD_DISPLAY_H

#include "display.h"
#include "i2c_lcd.h"
#include <string>
#include <driver/gpio.h>
#include <driver/i2c_master.h>
#include "freertos/semphr.h"
#include "freertos/queue.h"

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
    virtual void UpdateStatusBar(bool update_all = false) override;
    virtual void SetPowerSaveMode(bool on) override;
    void SetCursor(int row, int col);

protected:
    virtual bool Lock(int timeout_ms = 0) override { return true; }
    virtual void Unlock() override {}

private:
    struct DisplayMsg {
        char text[128];
        int row;
        int col;
    };
    static void DisplayTask(void* arg);
    void SendClear();
    void SendSetCursor(int row, int col);
    void SendShow(const char* text, int row, int col);
    void SendAnimation(const char* name, int row, int col);
    i2c_port_num_t i2c_port_;
    gpio_num_t sda_gpio_;
    gpio_num_t scl_gpio_;
    uint8_t i2c_addr_;
    int cols_;
    int rows_;

    QueueHandle_t display_queue_{nullptr};
    TaskHandle_t display_task_handle_{nullptr};
    int cursor_row_{0};
    int cursor_col_{0};
};

#endif // CHAR_LCD_DISPLAY_H
