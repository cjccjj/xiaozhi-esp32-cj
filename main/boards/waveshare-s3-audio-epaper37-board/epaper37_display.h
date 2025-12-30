#ifndef EPAPER37_DISPLAY_H
#define EPAPER37_DISPLAY_H

#include <driver/gpio.h>
#include <driver/spi_master.h>
#include "lcd_display.h"

/* Display color */
typedef enum {
    DRIVER_COLOR_WHITE  = 0xff,
    DRIVER_COLOR_BLACK  = 0x00,
} COLOR_IMAGE;

typedef struct {
    uint8_t cs;
    uint8_t dc;
    uint8_t rst;
    uint8_t busy;
    uint8_t mosi;
    uint8_t scl;
    int spi_host;
    int buffer_len;
} epaper37_spi_t;

#define PANEL_WIDTH 240
#define PANEL_HEIGHT 416

class Epaper37Display : public LcdDisplay {
public:
    Epaper37Display(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_handle_t panel,
                  int width, int height, int offset_x, int offset_y,
                  bool mirror_x, bool mirror_y, bool swap_xy, epaper37_spi_t _spi_data);
    ~Epaper37Display();

    void EPD_Init();
    void EPD_Update();
    void EPD_DeepSleep();
    void EPD_HW_RESET();
    void EPD_PartInit();
    void EPD_FastInit();
    void EPD_Display(const uint8_t *image);
    void EPD_Clear();
    bool EPD_DrawColorPixel(uint16_t x, uint16_t y, uint8_t color);

private:
    const epaper37_spi_t spi_data;
    const int Width;
    const int Height;
    const bool mirror_x;
    const bool mirror_y;
    const bool swap_xy;
    spi_device_handle_t spi;
    uint8_t *buffer = nullptr;
    uint8_t *old_buffer = nullptr;

    static void lvgl_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *color_p);

    void spi_gpio_init();
    void spi_port_init();
    void read_busy();

    void set_cs_1() { gpio_set_level((gpio_num_t)spi_data.cs, 1); }
    void set_cs_0() { gpio_set_level((gpio_num_t)spi_data.cs, 0); }
    void set_dc_1() { gpio_set_level((gpio_num_t)spi_data.dc, 1); }
    void set_dc_0() { gpio_set_level((gpio_num_t)spi_data.dc, 0); }
    void set_rst_1() { gpio_set_level((gpio_num_t)spi_data.rst, 1); }
    void set_rst_0() { gpio_set_level((gpio_num_t)spi_data.rst, 0); }

    void SPI_SendByte(uint8_t data);
    void EPD_SendData(uint8_t data);
    void EPD_SendCommand(uint8_t command);
    void writeBytes(const uint8_t *buffer, int len);
};

#endif // EPAPER37_DISPLAY_H
