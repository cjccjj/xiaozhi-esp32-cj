#include "dht20.h"

#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static const char* TAG = "DHT20";

static constexpr uint32_t MEASURE_DELAY_MS = 80;
static constexpr uint32_t MIN_INTERVAL_MS  = 1000;

Dht20::Dht20(i2c_master_bus_handle_t bus, uint8_t i2c_addr)
    : bus_(bus), addr_(i2c_addr)
{
}

Dht20::~Dht20()
{
    if (dev_) {
        i2c_master_bus_rm_device(dev_);
    }
}

esp_err_t Dht20::Init()
{
    i2c_device_config_t cfg = {};
    cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    cfg.device_address  = addr_;
    cfg.scl_speed_hz    = 100000;

    esp_err_t err =(i2c_master_bus_add_device(bus_, &cfg, &dev_));
    
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C device DHT20 failed to add");
        return err; // or handle
    }
    ESP_LOGI(TAG, "I2C device DHT20 Added");

    return ESP_OK;
}

esp_err_t Dht20::Read()
{
    int64_t now = esp_timer_get_time() / 1000;
    if (now - last_read_ms_ < MIN_INTERVAL_MS) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_ERROR_CHECK(TriggerMeasurement());

    vTaskDelay(pdMS_TO_TICKS(MEASURE_DELAY_MS));

    uint8_t data[7] = {};
    ESP_ERROR_CHECK(ReadMeasurement(data));

    if (data[0] & 0x80) {  // busy bit
        return ESP_ERR_TIMEOUT;
    }

    ParseMeasurement(data);
    last_read_ms_ = now;
    return ESP_OK;
}

float Dht20::GetTemperatureC() const
{
    return temperature_;
}

float Dht20::GetHumidity() const
{
    return humidity_;
}

// -------------------- private --------------------

esp_err_t Dht20::TriggerMeasurement()
{
    const uint8_t cmd[3] = {0xAC, 0x33, 0x00};
    return i2c_master_transmit(dev_, cmd, sizeof(cmd), -1);
}

esp_err_t Dht20::ReadMeasurement(uint8_t data[7])
{
    return i2c_master_receive(dev_, data, 7, -1);
}

void Dht20::ParseMeasurement(const uint8_t d[7])
{
    uint32_t raw_h =
        (uint32_t(d[1]) << 12) |
        (uint32_t(d[2]) << 4) |
        ((d[3] & 0xF0) >> 4);

    humidity_ = raw_h * 100.0f / 1048576.0f;

    uint32_t raw_t =
        (uint32_t(d[3] & 0x0F) << 16) |
        (uint32_t(d[4]) << 8) |
        d[5];

    temperature_ = raw_t * 200.0f / 1048576.0f - 50.0f;
}