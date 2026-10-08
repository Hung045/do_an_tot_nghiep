#include "CrashDetector.h"

//sợ ngày mai em đi mất chẳng còn thân xác xơ anh mot mau trong em
// so ngay mai anh ngu quen thuc giac tay om nham goi
// mot mau mua thay cho nang

#include <cmath>
#include <cstdint>

#include "BoardConfig.h"
#include "driver/i2c.h"
#include "freertos/task.h"

namespace {

constexpr i2c_port_t kI2cPort = I2C_NUM_0;
constexpr uint8_t kWhoAmIRegister = 0x75;
constexpr uint8_t kAccelDataRegister = 0x3B;
constexpr uint8_t kPowerManagementRegister = 0x6B;
constexpr uint32_t kAccelerometerCountsPerG = 16384;
constexpr float kRadiansToDegrees = 180.0F / 3.14159265F;
constexpr float kCrashTiltDegrees = 60.0F;
constexpr uint32_t kCrashConfirmMs = 200;

esp_err_t writeRegister(uint8_t registerAddress, uint8_t value)
{
    const uint8_t data[] = {registerAddress, value};
    return i2c_master_write_to_device(
        kI2cPort,
        DEMO_MPU6050_ADDRESS,
        data,
        sizeof(data),
        pdMS_TO_TICKS(100));
}

} // namespace

esp_err_t CrashDetector::initialize()
{
    i2c_config_t config = {};
    config.mode = I2C_MODE_MASTER;
    config.sda_io_num = DEMO_I2C_SDA_GPIO;
    config.scl_io_num = DEMO_I2C_SCL_GPIO;
    config.sda_pullup_en = GPIO_PULLUP_ENABLE;
    config.scl_pullup_en = GPIO_PULLUP_ENABLE;
    config.master.clk_speed = DEMO_I2C_FREQUENCY_HZ;

    esp_err_t err = i2c_param_config(kI2cPort, &config);
    if (err != ESP_OK) {
        return err;
    }

    err = i2c_driver_install(kI2cPort, config.mode, 0, 0, 0);
    if (err != ESP_OK) {
        return err;
    }

    uint8_t deviceId = 0;
    err = i2c_master_write_read_device(
        kI2cPort,
        DEMO_MPU6050_ADDRESS,
        &kWhoAmIRegister,
        1,
        &deviceId,
        1,
        pdMS_TO_TICKS(100));
    if (err != ESP_OK) {
        return err;
    }
    if ((deviceId & 0x7EU) != 0x68U) {
        return ESP_ERR_NOT_FOUND;
    }

    return writeRegister(kPowerManagementRegister, 0x00);
}

esp_err_t CrashDetector::update(float *tiltDegrees, bool *crashDetected)
{
    if (tiltDegrees == nullptr || crashDetected == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = readTiltDegrees(tiltDegrees);
    if (err != ESP_OK) {
        return err;
    }

    const TickType_t now = xTaskGetTickCount();
    if (*tiltDegrees >= kCrashTiltDegrees) {
        if (!tiltTiming_) {
            tiltTiming_ = true;
            tiltDetectedAt_ = now;
        }
        *crashDetected = (now - tiltDetectedAt_) >= pdMS_TO_TICKS(kCrashConfirmMs);
    } else {
        tiltTiming_ = false;
        *crashDetected = false;
    }

    return ESP_OK;
}

esp_err_t CrashDetector::readTiltDegrees(float *tiltDegrees)
{
    uint8_t data[6] = {};
    esp_err_t err = i2c_master_write_read_device(
        kI2cPort,
        DEMO_MPU6050_ADDRESS,
        &kAccelDataRegister,
        1,
        data,
        sizeof(data),
        pdMS_TO_TICKS(100));
    if (err != ESP_OK) {
        return err;
    }

    const int16_t accelX = static_cast<int16_t>(
        (static_cast<uint16_t>(data[0]) << 8) | data[1]);
    const int16_t accelY = static_cast<int16_t>(
        (static_cast<uint16_t>(data[2]) << 8) | data[3]);
    const int16_t accelZ = static_cast<int16_t>(
        (static_cast<uint16_t>(data[4]) << 8) | data[5]);

    const float xG = static_cast<float>(accelX) / kAccelerometerCountsPerG;
    const float yG = static_cast<float>(accelY) / kAccelerometerCountsPerG;
    const float zG = static_cast<float>(accelZ) / kAccelerometerCountsPerG;
    *tiltDegrees = std::atan2(std::sqrt((xG * xG) + (yG * yG)), zG) *
                   kRadiansToDegrees;
    return ESP_OK;
}
