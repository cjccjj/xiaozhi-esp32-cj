#include "epaper37_display.h"
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <string.h>
#include "board.h"
#include "config.h"
#include "esp_lvgl_port.h"
#include "lvgl_theme.h"

#define TAG "Epaper37Display"

#define BYTES_PER_PIXEL (LV_COLOR_FORMAT_GET_SIZE(LV_COLOR_FORMAT_RGB565))
#define BUFF_SIZE (PANEL_WIDTH * PANEL_HEIGHT * BYTES_PER_PIXEL)
#define PANEL_BUFFER_SIZE (PANEL_WIDTH * PANEL_HEIGHT / 8)

void Epaper37Display::lvgl_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *color_p) {
    Epaper37Display *driver = (Epaper37Display *)lv_display_get_user_data(disp);
    uint16_t         *pixel_p = (uint16_t *) color_p;
    bool changed = false;
    
    for (int y = area->y1; y <= area->y2; y++) {
        for (int x = area->x1; x <= area->x2; x++) {
            uint8_t color = (*pixel_p < 0x7fff) ? DRIVER_COLOR_BLACK : DRIVER_COLOR_WHITE; // %50 bright            
            //uint8_t color = (*pixel_p <  0xFC00) ? DRIVER_COLOR_BLACK : DRIVER_COLOR_WHITE; //99% 
            if (driver->EPD_DrawColorPixel(x, y, color)) {
                changed = true;
            }
            pixel_p++;
        }
    }

    // Only refresh the physical display if something actually changed
    if (changed) {
        driver->EPD_PartInit();
        driver->EPD_Display(driver->buffer);
    }
    
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

    // Current image buffer - Always PANEL_BUFFER_SIZE
    buffer = (uint8_t *)heap_caps_malloc(PANEL_BUFFER_SIZE, MALLOC_CAP_SPIRAM);
    assert(buffer);
    memset(buffer, 0xFF, PANEL_BUFFER_SIZE);

    // Old image buffer for partial refresh (required by 3.7" hardware)
    old_buffer = (uint8_t *)heap_caps_malloc(PANEL_BUFFER_SIZE, MALLOC_CAP_SPIRAM);
    assert(old_buffer);
    memset(old_buffer, 0xFF, PANEL_BUFFER_SIZE);

    display_ = lv_display_create(width, height);
    lv_display_set_flush_cb(display_, lvgl_flush_cb);
    lv_display_set_user_data(display_, this);

    // Use a full-screen RGB565 buffer for LVGL to render into (Width x Height)
    size_t lvgl_buffer_size = Width * Height * BYTES_PER_PIXEL;
    uint8_t *lvgl_buffer = (uint8_t *)heap_caps_malloc(lvgl_buffer_size, MALLOC_CAP_SPIRAM);
    assert(lvgl_buffer);
    lv_display_set_buffers(display_, lvgl_buffer, NULL, lvgl_buffer_size, LV_DISPLAY_RENDER_MODE_FULL);

    ESP_LOGI(TAG, "EPD init (Full Slow Refresh)");
    EPD_Init();
    EPD_Clear(); // Performs full refresh to clear screen
    EPD_PartInit(); // Switch to partial mode for LVGL updates

    lvgl_port_unlock();

    if (display_ == nullptr) {
        ESP_LOGE(TAG, "Failed to add display");
        return;
    }

    ESP_LOGI(TAG, "UI start");
    SetupUI();

    // Standard style layout customizations
    {
        DisplayLockGuard lock(this);
        
        // 1. Top bar: WiFi far right, Mute/Battery to its left
        if (top_bar_ && network_label_ && mute_label_) {
            lv_obj_t* right_icons = lv_obj_get_parent(mute_label_);
            if (right_icons) {
                // Ensure flex alignment pushes everything to the right
                lv_obj_set_flex_align(top_bar_, LV_FLEX_ALIGN_END, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
                
                // Order: right_icons (Mute/Battery) then network_label_ (WiFi)
                lv_obj_move_to_index(right_icons, 0);
                lv_obj_move_to_index(network_label_, 1);
                
                // Add spacing between the two groups if needed
                auto lvgl_theme = static_cast<LvglTheme*>(current_theme_);
                lv_obj_set_style_margin_left(network_label_, lvgl_theme->spacing(2), 0);
            }
        }

        // 2. Status overlay: Align to top-left of screen with same padding as top_bar
        if (status_bar_) {
            auto lvgl_theme = static_cast<LvglTheme*>(current_theme_);
            lv_obj_set_style_pad_top(status_bar_, lvgl_theme->spacing(2), 0);
            lv_obj_set_style_pad_bottom(status_bar_, lvgl_theme->spacing(2), 0);
            lv_obj_set_style_pad_left(status_bar_, lvgl_theme->spacing(4), 0);
            lv_obj_set_style_pad_right(status_bar_, lvgl_theme->spacing(4), 0);

            lv_obj_set_style_align(status_bar_, LV_ALIGN_TOP_LEFT, 0);
            lv_obj_align(status_bar_, LV_ALIGN_TOP_LEFT, 0, 0);
            
            // Adjust label alignments inside status_bar_
            if (status_label_) {
                lv_obj_set_style_text_align(status_label_, LV_TEXT_ALIGN_LEFT, 0);
                lv_obj_align(status_label_, LV_ALIGN_LEFT_MID, 0, 0);
            }
            if (notification_label_) {
                lv_obj_set_style_text_align(notification_label_, LV_TEXT_ALIGN_LEFT, 0);
                lv_obj_align(notification_label_, LV_ALIGN_LEFT_MID, 0, 0);
            }
        }

        // 3. Emoji Box: 100x100, below top_bar, centered content
        if (emoji_box_) {
            lv_obj_set_size(emoji_box_, 100, 100);
            lv_obj_align_to(emoji_box_, top_bar_, LV_ALIGN_OUT_BOTTOM_MID, 0, 0);
            
            if (emoji_image_) {
                lv_obj_center(emoji_image_);
            }
            if (emoji_label_) {
                lv_obj_set_style_text_align(emoji_label_, LV_TEXT_ALIGN_CENTER, 0);
                lv_obj_center(emoji_label_);
            }
        }
    }
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
    ESP_LOGI(TAG, "EPD Full Init");
    EPD_HW_RESET();
    read_busy();

    EPD_SendCommand(0x00);
    EPD_SendData(0x1B);
}

void Epaper37Display::EPD_Update() {
    ESP_LOGI(TAG, "EPD Refreshing Display...");
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
    ESP_LOGI(TAG, "EPD Partial Init");
    EPD_HW_RESET();
    read_busy();

    EPD_SendCommand(0x00);
    EPD_SendData(0x1B);
    EPD_SendCommand(0xE0);
    EPD_SendData(0x02);
    EPD_SendCommand(0xE5);
    EPD_SendData(0x6E);
}

void Epaper37Display::EPD_FastInit() {
    ESP_LOGI(TAG, "EPD Fast Init");
    EPD_HW_RESET();
    read_busy();

    EPD_SendCommand(0x00);
    EPD_SendData(0x1B);
    EPD_SendCommand(0xE0);
    EPD_SendData(0x02);
    EPD_SendCommand(0xE5);
    EPD_SendData(0x5F);
}

void Epaper37Display::EPD_Display(const uint8_t *image) {
    int data_len = PANEL_WIDTH * PANEL_HEIGHT / 8;
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
    ESP_LOGI(TAG, "EPD Clearing Screen (Full)");
    int data_len = PANEL_WIDTH * PANEL_HEIGHT / 8;
    // Clear hardware display to white
    memset(buffer, 0xFF, PANEL_BUFFER_SIZE);
    
    // According to STM32 driver EPD_Display_Clear:
    // It sends oldImage to 0x10 and 0xFF to 0x13
    EPD_SendCommand(0x10);
    writeBytes(old_buffer, data_len);
    
    EPD_SendCommand(0x13);
    writeBytes(buffer, data_len);
    
    EPD_Update();
    
    // Sync local buffers
    memset(old_buffer, 0xFF, PANEL_BUFFER_SIZE);
    memset(buffer, 0xFF, PANEL_BUFFER_SIZE);
}

bool Epaper37Display::EPD_DrawColorPixel(uint16_t x, uint16_t y, uint8_t color) {
    // LVGL coordinates (x, y) where x is [0, 415] and y is [0, 239]
    // We want 90-degree clockwise rotation to physical panel (240x416)
    // Physical X = (PANEL_WIDTH - 1) - y = 239 - y
    // Physical Y = x
    
    if (x >= Width || y >= Height) return false;

    uint16_t _x = (PANEL_WIDTH - 1) - y;
    uint16_t _y = x;

    // Apply any additional mirroring if specified in config
    if (mirror_x) {
        _x = (PANEL_WIDTH - 1) - _x;
    }
    if (mirror_y) {
        _y = (PANEL_HEIGHT - 1) - _y;
    }
    if (swap_xy) {
        uint16_t tmp = _x;
        _x = _y;
        _y = tmp;
    }

    if (_x >= PANEL_WIDTH || _y >= PANEL_HEIGHT) return false;

    // 240x416 resolution. Physical Width is 240 bits (30 bytes).
    // index = y * (PANEL_WIDTH/8) + (x/8)
    uint16_t index = _y * (PANEL_WIDTH / 8) + (_x / 8);
    uint8_t bit = 7 - (_x % 8);

    uint8_t old_val = buffer[index];
    if (color == DRIVER_COLOR_WHITE) {
        buffer[index] |= (1 << bit);
    } else {
        buffer[index] &= ~(1 << bit);
    }

    return old_val != buffer[index];
}
