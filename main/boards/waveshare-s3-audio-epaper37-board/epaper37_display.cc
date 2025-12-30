#include "epaper37_display.h"
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <string.h>
#include "board.h"
#include "config.h"
#include "esp_lvgl_port.h"

#define TAG "Epaper37Display"

#define BYTES_PER_PIXEL (LV_COLOR_FORMAT_GET_SIZE(LV_COLOR_FORMAT_RGB565))
#define BUFF_SIZE (EXAMPLE_LCD_WIDTH * EXAMPLE_LCD_HEIGHT * BYTES_PER_PIXEL)

void Epaper37Display::lvgl_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *color_p) {
    Epaper37Display *driver = (Epaper37Display *)lv_display_get_user_data(disp);
    uint16_t *rgb_buffer = (uint16_t *)color_p;

    // According to STM32 example main.c:
    // while(1) { EPD_PartInit(); ... EPD_Display(); EPD_Update(); delay; }
    // We call EPD_PartInit() here to ensure the hardware is in the correct state for partial update.
    driver->EPD_PartInit();
    
    // Convert RGB565 to 1-bit B/W using luminance threshold
    for (int y = 0; y < driver->Height; y++) {
        for (int x = 0; x < driver->Width; x++) {
            uint16_t color = rgb_buffer[y * driver->Width + x];
            
            // Extract RGB components from RGB565
            uint8_t r = (color >> 11) & 0x1F; // 5 bits
            uint8_t g = (color >> 5) & 0x3F;  // 6 bits
            uint8_t b = color & 0x1F;         // 5 bits
            
            // Convert to 0-255 range
            uint16_t r8 = (r * 255) / 31;
            uint16_t g8 = (g * 255) / 63;
            uint16_t b8 = (b * 255) / 31;
            
            // Calculate luminance: Y = 0.299R + 0.587G + 0.114B
            // Integer math: (r*76 + g*150 + b*29) >> 8
            uint16_t luminance = (r8 * 76 + g8 * 150 + b8 * 29) >> 8;
            
            // Use 128 as threshold for B/W
            uint8_t bw_color = (luminance > 128) ? DRIVER_COLOR_WHITE : DRIVER_COLOR_BLACK;
            driver->EPD_DrawColorPixel(x, y, bw_color);
        }
    }

    // Refresh the display (EPD_Display calls EPD_Update)
    driver->EPD_Display(driver->buffer);
    
    lv_display_flush_ready(disp);
}

Epaper37Display::Epaper37Display(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_handle_t panel,
                               int width, int height, int offset_x, int offset_y,
                               bool _mirror_x, bool _mirror_y, bool _swap_xy, epaper37_spi_t _spi_data)
    : LcdDisplay(panel_io, panel, width, height), spi_data(_spi_data), 
      Width(width), Height(height), mirror_x(_mirror_x), mirror_y(_mirror_y), swap_xy(_swap_xy) {

    ESP_LOGI(TAG, "Initialize SPI");
    spi_port_init();
    spi_gpio_init();

    ESP_LOGI(TAG, "Initialize LVGL library");
    lv_init();

    lvgl_port_cfg_t port_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    port_cfg.task_priority = 2;
    port_cfg.timer_period_ms = 50;
    lvgl_port_init(&port_cfg);
    lvgl_port_lock(0);

    // Current image buffer
    buffer = (uint8_t *)heap_caps_malloc(spi_data.buffer_len, MALLOC_CAP_SPIRAM);
    assert(buffer);
    memset(buffer, 0xFF, spi_data.buffer_len);

    // Old image buffer for partial refresh (required by 3.7" hardware)
    old_buffer = (uint8_t *)heap_caps_malloc(spi_data.buffer_len, MALLOC_CAP_SPIRAM);
    assert(old_buffer);
    memset(old_buffer, 0xFF, spi_data.buffer_len);

    display_ = lv_display_create(width, height);
    lv_display_set_flush_cb(display_, lvgl_flush_cb);
    lv_display_set_user_data(display_, this);

    // Use a full-screen RGB565 buffer for LVGL to render into
    uint8_t *lvgl_buffer = (uint8_t *)heap_caps_malloc(BUFF_SIZE, MALLOC_CAP_SPIRAM);
    assert(lvgl_buffer);
    lv_display_set_buffers(display_, lvgl_buffer, NULL, BUFF_SIZE, LV_DISPLAY_RENDER_MODE_FULL);

    ESP_LOGI(TAG, "EPD init");
    EPD_Init();
    EPD_Clear(); // Clear both hardware and local buffers
    EPD_PartInit(); // Use partial refresh mode for subsequent updates

    lvgl_port_unlock();

    if (display_ == nullptr) {
        ESP_LOGE(TAG, "Failed to add display");
        return;
    }

    ESP_LOGI(TAG, "UI start");
    SetupUI();
}

Epaper37Display::~Epaper37Display() {
    if (buffer) free(buffer);
    if (old_buffer) free(old_buffer);
}

void Epaper37Display::spi_gpio_init() {
    gpio_config_t gpio_conf = {};
    gpio_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_conf.mode = GPIO_MODE_OUTPUT;
    gpio_conf.pin_bit_mask = (1ULL << spi_data.rst) | (1ULL << spi_data.dc) | (1ULL << spi_data.cs);
    gpio_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    ESP_ERROR_CHECK(gpio_config(&gpio_conf));

    gpio_conf.mode = GPIO_MODE_INPUT;
    gpio_conf.pin_bit_mask = (1ULL << spi_data.busy);
    ESP_ERROR_CHECK(gpio_config(&gpio_conf));

    set_rst_1();
}

void Epaper37Display::spi_port_init() {
    esp_err_t ret;
    spi_bus_config_t buscfg = {};
    buscfg.miso_io_num = -1;
    buscfg.mosi_io_num = spi_data.mosi;
    buscfg.sclk_io_num = spi_data.scl;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = spi_data.buffer_len;

    spi_device_interface_config_t devcfg = {};
    devcfg.spics_io_num = -1; // We handle CS manually
    devcfg.clock_speed_hz = 20 * 1000 * 1000;
    devcfg.mode = 0;
    devcfg.queue_size = 7;

    ret = spi_bus_initialize((spi_host_device_t)spi_data.spi_host, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);
    ret = spi_bus_add_device((spi_host_device_t)spi_data.spi_host, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);
}

void Epaper37Display::read_busy() {
    // STM32 code: while(EPD_ReadBusy==0);
    // This means it waits for BUSY to become 1 (idle).
    while (gpio_get_level((gpio_num_t)spi_data.busy) == 0) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void Epaper37Display::SPI_SendByte(uint8_t data) {
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 8;
    t.tx_buffer = &data;
    spi_device_polling_transmit(spi, &t);
}

void Epaper37Display::EPD_SendData(uint8_t data) {
    set_dc_1();
    set_cs_0();
    SPI_SendByte(data);
    set_cs_1();
}

void Epaper37Display::EPD_SendCommand(uint8_t command) {
    set_dc_0();
    set_cs_0();
    SPI_SendByte(command);
    set_cs_1();
}

void Epaper37Display::writeBytes(const uint8_t *data, int len) {
    if (len <= 0) return;
    set_dc_1();
    set_cs_0();
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 8 * len;
    t.tx_buffer = data;
    spi_device_polling_transmit(spi, &t);
    set_cs_1();
}

void Epaper37Display::EPD_HW_RESET() {
    vTaskDelay(pdMS_TO_TICKS(100));
    set_rst_0();
    vTaskDelay(pdMS_TO_TICKS(20));
    set_rst_1();
    vTaskDelay(pdMS_TO_TICKS(20));
    read_busy();
}

void Epaper37Display::EPD_Init() {
    EPD_HW_RESET();

    EPD_SendCommand(0x00);
    EPD_SendData(0x1B);
}

void Epaper37Display::EPD_Update() {
    EPD_SendCommand(0x04); // Power ON
    read_busy();
    EPD_SendCommand(0x12); // Display Refresh
    read_busy();
}

void Epaper37Display::EPD_DeepSleep() {
    EPD_SendCommand(0x02); // Power OFF
    read_busy();
    EPD_SendCommand(0x07); // Deep Sleep
    EPD_SendData(0xA5);
}

void Epaper37Display::EPD_PartInit() {
    EPD_HW_RESET();

    EPD_SendCommand(0x00);
    EPD_SendData(0x1B);
    EPD_SendCommand(0xE0);
    EPD_SendData(0x02);
    EPD_SendCommand(0xE5);
    EPD_SendData(0x6E);
}

void Epaper37Display::EPD_FastInit() {
    EPD_HW_RESET();

    EPD_SendCommand(0x00);
    EPD_SendData(0x1B);
    EPD_SendCommand(0xE0);
    EPD_SendData(0x02);
    EPD_SendCommand(0xE5);
    EPD_SendData(0x5F);
}

void Epaper37Display::EPD_Display(const uint8_t *image) {
    int data_len = Width * Height / 8;
    // Send old image
    EPD_SendCommand(0x10);
    writeBytes(old_buffer, data_len);

    // Send new image
    EPD_SendCommand(0x13);
    writeBytes(image, data_len);

    // Update hardware
    EPD_Update();

    // Update local old buffer
    memcpy(old_buffer, image, data_len);
}

void Epaper37Display::EPD_Clear() {
    int data_len = Width * Height / 8;
    // Clear hardware display to white
    memset(buffer, 0xFF, spi_data.buffer_len);
    
    // According to STM32 driver EPD_Display_Clear:
    // It sends oldImage to 0x10 and 0xFF to 0x13
    EPD_SendCommand(0x10);
    writeBytes(old_buffer, data_len);
    
    EPD_SendCommand(0x13);
    writeBytes(buffer, data_len);
    
    EPD_Update();
    
    // Sync local buffers
    memset(old_buffer, 0xFF, spi_data.buffer_len);
    memset(buffer, 0xFF, spi_data.buffer_len);
}

void Epaper37Display::EPD_DrawColorPixel(uint16_t x, uint16_t y, uint8_t color) {
    uint16_t _x = x;
    uint16_t _y = y;

    if (swap_xy) {
        uint16_t tmp = _x;
        _x = _y;
        _y = tmp;
    }

    if (mirror_x) {
        _x = Width - 1 - _x;
    }

    if (mirror_y) {
        _y = Height - 1 - _y;
    }

    if (_x >= Width || _y >= Height) return;

    // 240x416 resolution. Width is 240 bits (30 bytes).
    // index = y * (Width/8) + (x/8)
    uint16_t index = _y * (Width / 8) + (_x / 8);
    uint8_t bit = 7 - (_x % 8);

    if (color == DRIVER_COLOR_WHITE) {
        buffer[index] |= (1 << bit);
    } else {
        buffer[index] &= ~(1 << bit);
    }
}
