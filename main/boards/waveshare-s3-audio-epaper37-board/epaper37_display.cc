#include <stdio.h>
#include <esp_lcd_panel_io.h>
#include "epaper37_display.h"
#include <freertos/FreeRTOS.h>
#include <vector>
#include <esp_log.h>
#include "board.h"
#include "config.h"
#include "esp_lvgl_port.h"
#include "settings.h"
#include <string.h>

#define TAG "Epaper37"

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

Epaper37Display::Epaper37Display(esp_lcd_panel_io_handle_t panel_io,
                                 esp_lcd_panel_handle_t panel,
                                 int width,
                                 int height,
                                 int offset_x,
                                 int offset_y,
                                 bool mirror_x,
                                 bool mirror_y,
                                 bool swap_xy,
                                 epaper37_spi_t cfg)
    : LcdDisplay(panel_io, panel, width, height),
      spi_cfg(cfg),
      Width(width),
      Height(height)
{
    ESP_LOGI(TAG, "Initialize SPI");
    spi_port_init();
    spi_gpio_init();

    // Allocate e-paper 1-bit buffers
    buffer = (uint8_t *) heap_caps_malloc(spi_cfg.buffer_len, MALLOC_CAP_SPIRAM);
    old_buffer = (uint8_t *) heap_caps_malloc(spi_cfg.buffer_len, MALLOC_CAP_SPIRAM);

    assert(buffer);
    assert(old_buffer);

    memset(buffer, 0xFF, spi_cfg.buffer_len);
    memset(old_buffer, 0xFF, spi_cfg.buffer_len);

    ESP_LOGI(TAG, "Initialize LVGL");
    lv_init();

    // LVGL port setup
    lvgl_port_cfg_t port_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    port_cfg.task_priority = 2;
    port_cfg.timer_period_ms = 50;
    lvgl_port_init(&port_cfg);
    lvgl_port_lock(0);

    // LVGL display object
    display_ = lv_display_create(width, height);
    lv_display_set_flush_cb(display_, lvgl_flush_cb);
    lv_display_set_user_data(display_, this);

    // Allocate LVGL RGB565 framebuffer
    size_t fb_size = width * height * 2; // RGB565 = 2 bytes/pixel
    uint8_t *fb = (uint8_t *) heap_caps_malloc(fb_size, MALLOC_CAP_SPIRAM);
    assert(fb);

    lv_display_set_buffers(display_,
                           fb,
                           NULL,
                           fb_size,
                           LV_DISPLAY_RENDER_MODE_FULL);

    ESP_LOGI(TAG, "EPD HW init");
    EPD_Init();
    EPD_Clear();
    EPD_Update();

    lvgl_port_unlock();
}


// ---------------------------------------------------------------------------
// Destructor
// ---------------------------------------------------------------------------
Epaper37Display::~Epaper37Display() {
}


// ---------------------------------------------------------------------------
// SPI GPIO Initialization
// ---------------------------------------------------------------------------

void Epaper37Display::spi_gpio_init() {
    gpio_config_t io = {};
    io.intr_type = GPIO_INTR_DISABLE;
    io.mode = GPIO_MODE_OUTPUT;
    io.pull_up_en = GPIO_PULLUP_ENABLE;
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.pin_bit_mask = (1ULL << spi_cfg.cs) |
                      (1ULL << spi_cfg.dc) |
                      (1ULL << spi_cfg.rst);
    gpio_config(&io);

    io.mode = GPIO_MODE_INPUT;
    io.pin_bit_mask = (1ULL << spi_cfg.busy);
    gpio_config(&io);

    set_rst_1();
}


// ---------------------------------------------------------------------------
// SPI Bus Initialization
// ---------------------------------------------------------------------------

void Epaper37Display::spi_port_init() {
    spi_bus_config_t buscfg = {};
    buscfg.mosi_io_num = spi_cfg.mosi;
    buscfg.miso_io_num = -1;
    buscfg.sclk_io_num = spi_cfg.scl;
    buscfg.max_transfer_sz = spi_cfg.buffer_len + 64; // Safe size

    spi_bus_initialize((spi_host_device_t)spi_cfg.spi_host,
                       &buscfg,
                       SPI_DMA_CH_AUTO);

    spi_device_interface_config_t devcfg = {};
    devcfg.spics_io_num = -1;
    devcfg.clock_speed_hz = 20 * 1000 * 1000; // stable 20MHz
    devcfg.mode = 0;
    devcfg.queue_size = 7;

    spi_bus_add_device((spi_host_device_t)spi_cfg.spi_host,
                       &devcfg,
                       &spi);
}


// ---------------------------------------------------------------------------
// Busy wait
// ---------------------------------------------------------------------------

void Epaper37Display::read_busy() {
    while (gpio_get_level((gpio_num_t)spi_cfg.busy) == 0) {
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}


// ---------------------------------------------------------------------------
// SPI Commands
// ---------------------------------------------------------------------------

void Epaper37Display::sendCommand(uint8_t cmd) {
    set_dc_0();
    set_cs_0();
    spi_transaction_t t = {};
    t.length = 8;
    t.tx_buffer = &cmd;
    spi_device_polling_transmit(spi, &t);
    set_cs_1();
}

void Epaper37Display::sendData(uint8_t data) {
    set_dc_1();
    set_cs_0();
    spi_transaction_t t = {};
    t.length = 8;
    t.tx_buffer = &data;
    spi_device_polling_transmit(spi, &t);
    set_cs_1();
}

void Epaper37Display::writeBytes(const uint8_t *data, int len) {
    set_dc_1();
    set_cs_0();
    spi_transaction_t t = {};
    t.length = len * 8;
    t.tx_buffer = data;
    spi_device_polling_transmit(spi, &t);
    set_cs_1();
}


// ---------------------------------------------------------------------------
// E-Ink Driver Logic (from STM32 sample, correct for this panel)
// ---------------------------------------------------------------------------

void Epaper37Display::EPD_Init() {
    set_rst_1(); vTaskDelay(pdMS_TO_TICKS(100));
    set_rst_0(); vTaskDelay(pdMS_TO_TICKS(20));
    set_rst_1(); vTaskDelay(pdMS_TO_TICKS(20));

    read_busy();

    sendCommand(0x00);
    sendData(0x1B);
}

void Epaper37Display::EPD_FastInit() {
    set_rst_1(); vTaskDelay(pdMS_TO_TICKS(100));
    set_rst_0(); vTaskDelay(pdMS_TO_TICKS(20));
    set_rst_1(); vTaskDelay(pdMS_TO_TICKS(20));
    read_busy();

    sendCommand(0x00);
    sendData(0x1B);

    sendCommand(0xE0);
    sendData(0x02);

    sendCommand(0xE5);
    sendData(0x5F);
}

void Epaper37Display::EPD_PartInit() {
    set_rst_1(); vTaskDelay(pdMS_TO_TICKS(100));
    set_rst_0(); vTaskDelay(pdMS_TO_TICKS(20));
    set_rst_1(); vTaskDelay(pdMS_TO_TICKS(20));
    read_busy();

    sendCommand(0x00);
    sendData(0x1B);

    sendCommand(0xE0);
    sendData(0x02);

    sendCommand(0xE5);
    sendData(0x6E);
}


void Epaper37Display::EPD_Clear() {
    int W = (Width + 7) / 8;
    int H = Height;
    int size = W * H;

    sendCommand(0x10);
    writeBytes(old_buffer, size);

    memset(buffer, 0xFF, size);
    sendCommand(0x13);
    writeBytes(buffer, size);

    memcpy(old_buffer, buffer, size);
}


void Epaper37Display::EPD_Display() {
    int size = spi_cfg.buffer_len;

    sendCommand(0x10);
    writeBytes(old_buffer, size);

    sendCommand(0x13);
    writeBytes(buffer, size);

    memcpy(old_buffer, buffer, size);
}


void Epaper37Display::EPD_Update() {
    sendCommand(0x04);
    read_busy();

    sendCommand(0x12);
    read_busy();
}


void Epaper37Display::EPD_DeepSleep() {
    sendCommand(0x02);
    read_busy();
    sendCommand(0x07);
    sendData(0xA5);
}


// ---------------------------------------------------------------------------
// DrawPixel (1-bit)
// ---------------------------------------------------------------------------

void Epaper37Display::EPD_DrawColorPixel(uint16_t x, uint16_t y, uint8_t color) {
    if (x >= Width || y >= Height) return;

    int rowBytes = (Width + 7) / 8;

    uint32_t index = y * rowBytes + (x >> 3);
    uint8_t bit = 7 - (x & 7);

    if (color)
        buffer[index] |= (1 << bit);
    else
        buffer[index] &= ~(1 << bit);
}


// ---------------------------------------------------------------------------
// LVGL Flush Callback
// ---------------------------------------------------------------------------

void Epaper37Display::lvgl_flush_cb(lv_display_t *disp,
                                    const lv_area_t *area,
                                    uint8_t *color_p)
{
    Epaper37Display *driver =
        (Epaper37Display *) lv_display_get_user_data(disp);

    driver->EPD_Display();
    driver->EPD_Update();

    lv_disp_flush_ready(disp);
}