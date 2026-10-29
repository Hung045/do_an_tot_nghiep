#include "CrashDetector.h"

#include <cmath>
#include <cstdint>

#include "BoardConfig.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/task.h"

namespace {

constexpr i2c_port_t kI2cPort = I2C_NUM_0;
constexpr uint8_t kWhoAmIRegister = 0x75;
constexpr uint8_t kConfigRegister = 0x1A; 
constexpr uint8_t kAccelDataRegister = 0x3B;
constexpr uint8_t kPowerManagementRegister = 0x6B;
constexpr float kRadiansToDegrees = 180.0F / 3.14159265F;
constexpr float kCrashTiltDegrees = 60.0F;
constexpr uint32_t kCrashConfirmMs = 200;

esp_err_t writeRegister(uint8_t registerAddress, uint8_t value)
{
    BoardI2c::LockGuard lock(pdMS_TO_TICKS(100));
    if (!lock.isLocked()) {
        return ESP_ERR_TIMEOUT;
    }

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
    BoardI2c::getMutex();

    uint8_t deviceId = 0;
    esp_err_t err = ESP_FAIL;
    {
        BoardI2c::LockGuard lock(pdMS_TO_TICKS(100));
        if (!lock.isLocked()) {
            available_ = false;
            return ESP_ERR_TIMEOUT;
        }
        err = i2c_master_write_read_device(
            kI2cPort,
            DEMO_MPU6050_ADDRESS,
            &kWhoAmIRegister,
            1,
            &deviceId,
            1,
            pdMS_TO_TICKS(100));
    }

    if (err != ESP_OK) {
        available_ = false;
        ESP_LOGW("CrashDetector", "MPU6050 not responding on I2C bus; continuing without crash detection");
        return ESP_OK;
    }
    if ((deviceId & 0x7EU) != 0x68U) {
        available_ = false;
        ESP_LOGW("CrashDetector", "MPU6050 not detected at address 0x%02X; continuing without crash detection",
                 DEMO_MPU6050_ADDRESS);
        return ESP_OK;
    }

    err = writeRegister(kPowerManagementRegister, 0x00);
    if (err != ESP_OK) {
        available_ = false;
        return err;
    }
    err = writeRegister(kConfigRegister, 0x05);
    if (err != ESP_OK) {
        available_ = false;
        return err;
    }

    available_ = true;
    return ESP_OK;
}

esp_err_t CrashDetector::update(float *tiltDegrees, bool *crashDetected)
{
    if (tiltDegrees == nullptr || crashDetected == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!available_) {
        *tiltDegrees = 0.0F;
        *crashDetected = false;
        return ESP_OK;
    }

    esp_err_t err = readTiltDegrees(tiltDegrees);
    if (err != ESP_OK) {
        return err;
    }

    const TickType_t now = xTaskGetTickCount();
    if (std::fabs(*tiltDegrees) >= kCrashTiltDegrees) {
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
    {
        BoardI2c::LockGuard lock(pdMS_TO_TICKS(100));
        if (!lock.isLocked()) {
            return ESP_ERR_TIMEOUT;
        }
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
    }

    const auto accelX = static_cast<float>(static_cast<int16_t>((static_cast<uint16_t>(data[0]) << 8) | data[1]));
    const auto accelY = static_cast<float>(static_cast<int16_t>((static_cast<uint16_t>(data[2]) << 8) | data[3]));
    const auto accelZ = static_cast<float>(static_cast<int16_t>((static_cast<uint16_t>(data[4]) << 8) | data[5]));

    *tiltDegrees = std::atan2(std::sqrt((accelX * accelX) + (accelY * accelY)), std::fabs(accelZ)) *
                   kRadiansToDegrees;
    return ESP_OK;
}