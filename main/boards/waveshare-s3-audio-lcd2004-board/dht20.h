#ifndef DHT20_H
#define DHT20_H

#include <esp_err.h>
#include <driver/i2c_master.h>

class Dht20 {
public:
    // Board passes the already-created I2C bus
    explicit Dht20(i2c_master_bus_handle_t bus,
                   uint8_t i2c_addr = 0x38);

    ~Dht20();

    // Minimal public API (as you requested)
    esp_err_t Init();
    esp_err_t Read();

    float GetTemperatureC() const;
    float GetHumidity() const;

private:
    // --- internal helpers ---
    esp_err_t TriggerMeasurement();
    esp_err_t ReadMeasurement(uint8_t data[7]);
    void ParseMeasurement(const uint8_t data[7]);

private:
    i2c_master_bus_handle_t bus_;
    i2c_master_dev_handle_t dev_{};

    uint8_t addr_;
    float temperature_{0.0f};
    float humidity_{0.0f};
    int64_t last_read_ms_{0};
};

#endif // DHT20_H