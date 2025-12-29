#ifndef __EPAPER37_DISPLAY_H__
#define __EPAPER37_DISPLAY_H__

#include <driver/gpio.h>
#include "lcd_display.h"

typedef struct {
    uint8_t cs;//3
    uint8_t dc;//7
    uint8_t rst;//5
    uint8_t busy;//6
    uint8_t mosi;//9
    uint8_t scl;//4
    int spi_host;
    int buffer_len;
} epaper37_spi_t;

class Epaper37Display : public LcdDisplay {
public:
    Epaper37Display(esp_lcd_panel_io_handle_t panel_io,
                    esp_lcd_panel_handle_t panel,
                    int width,
                    int height,
                    int offset_x,
                    int offset_y,
                    bool mirror_x,
                    bool mirror_y,
                    bool swap_xy,
                    epaper37_spi_t cfg);

    ~Epaper37Display();

    void EPD_Init();
    void EPD_FastInit();
    void EPD_PartInit();
    void EPD_Clear();
    void EPD_Display();
    void EPD_Update();
    void EPD_DeepSleep();

    void EPD_DrawColorPixel(uint16_t x, uint16_t y, uint8_t color);

private:
    static void lvgl_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *color_p);

    void spi_port_init();
    void spi_gpio_init();
    void read_busy();

    void sendCommand(uint8_t cmd);
    void sendData(uint8_t data);
    void writeBytes(const uint8_t *data, int len);

    inline void set_cs_1() { gpio_set_level((gpio_num_t)spi_cfg.cs, 1); }
    inline void set_cs_0() { gpio_set_level((gpio_num_t)spi_cfg.cs, 0); }
    inline void set_dc_1() { gpio_set_level((gpio_num_t)spi_cfg.dc, 1); }
    inline void set_dc_0() { gpio_set_level((gpio_num_t)spi_cfg.dc, 0); }
    inline void set_rst_1(){ gpio_set_level((gpio_num_t)spi_cfg.rst, 1); }
    inline void set_rst_0(){ gpio_set_level((gpio_num_t)spi_cfg.rst, 0); }

private:
    const epaper37_spi_t spi_cfg;
    const int Width;
    const int Height;

    spi_device_handle_t spi;
    uint8_t *buffer = NULL;      // Current frame (1-bit)
    uint8_t *old_buffer = NULL;  // Previous frame (1-bit)
};

#endif