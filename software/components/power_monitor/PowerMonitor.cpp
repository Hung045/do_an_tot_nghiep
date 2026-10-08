#include "PowerMonitor.h"

#include <cstdint>

#include "BoardConfig.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {

constexpr uint8_t kConfigurationRegister = 0x00;
constexpr uint8_t kBusVoltageRegister = 0x02;
constexpr uint8_t kPowerRegister = 0x03;
constexpr uint8_t kCurrentRegister = 0x04;
constexpr uint8_t kCalibrationRegister = 0x05;
constexpr uint16_t kContinuousShuntAndBusConfig = 0x4527;
constexpr float kMaximumShuntVoltageVolts = 0.08192F;

esp_err_t writeRegister(uint8_t address, uint16_t value)
{
    const uint8_t data[] = {
        address,
        static_cast<uint8_t>(value >> 8),
        static_cast<uint8_t>(value & 0xFFU),
    };
    return i2c_master_write_to_device(
        I2C_NUM_0,
        DEMO_INA226_ADDRESS,
        data,
        sizeof(data),
        pdMS_TO_TICKS(100));
}

esp_err_t readRegister(uint8_t address, uint16_t *value)
{
    if (value == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t data[2] = {};
    esp_err_t err = i2c_master_write_read_device(
        I2C_NUM_0,
        DEMO_INA226_ADDRESS,
        &address,
        1,
        data,
        sizeof(data),
        pdMS_TO_TICKS(100));
    if (err != ESP_OK) {
        return err;
    }

    *value = (static_cast<uint16_t>(data[0]) << 8) | data[1];
    return ESP_OK;
}

} // namespace

esp_err_t PowerMonitor::initialize()
{
    constexpr float shuntResistanceOhms =
        static_cast<float>(DEMO_INA226_SHUNT_MICRO_OHMS) / 1000000.0F;
    constexpr float maximumCurrentAmps =
        static_cast<float>(DEMO_INA226_MAX_CURRENT_MILLIAMPS) / 1000.0F;

    if (shuntResistanceOhms <= 0.0F || maximumCurrentAmps <= 0.0F ||
        shuntResistanceOhms * maximumCurrentAmps > kMaximumShuntVoltageVolts) {
        return ESP_ERR_INVALID_ARG;
    }

    currentLsbAmps_ = maximumCurrentAmps / 32768.0F;
    const float calibrationValue =
        0.00512F / (currentLsbAmps_ * shuntResistanceOhms);
    if (calibrationValue < 1.0F || calibrationValue > 65535.0F) {
        currentLsbAmps_ = 0.0F;
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = writeRegister(kConfigurationRegister, 0x8000U);
    if (err != ESP_OK) {
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(2));

    err = writeRegister(kConfigurationRegister, kContinuousShuntAndBusConfig);
    if (err != ESP_OK) {
        return err;
    }
    err = writeRegister(kCalibrationRegister, static_cast<uint16_t>(calibrationValue));
    if (err != ESP_OK) {
        return err;
    }

    uint16_t deviceId = 0;
    err = readRegister(0xFF, &deviceId);
    if (err != ESP_OK) {
        return err;
    }
    if (deviceId != 0x2260U && deviceId != 0x2261U) {
        return ESP_ERR_NOT_FOUND;
    }

    return ESP_OK;
}

esp_err_t PowerMonitor::read(PowerReading *reading)
{
    if (reading == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    if (currentLsbAmps_ <= 0.0F) {
        return ESP_ERR_INVALID_STATE;
    }

    uint16_t busRaw = 0;
    uint16_t currentRaw = 0;
    uint16_t powerRaw = 0;

    esp_err_t err = readRegister(kBusVoltageRegister, &busRaw);
    if (err != ESP_OK) {
        return err;
    }
    err = readRegister(kCurrentRegister, &currentRaw);
    if (err != ESP_OK) {
        return err;
    }
    err = readRegister(kPowerRegister, &powerRaw);
    if (err != ESP_OK) {
        return err;
    }

    const int16_t signedCurrent = static_cast<int16_t>(currentRaw);
    reading->busVoltageVolts = static_cast<float>(busRaw) * 0.00125F;
    reading->currentMilliamps =
        static_cast<float>(signedCurrent) * currentLsbAmps_ * 1000.0F;
    reading->powerMilliwatts =
        static_cast<float>(powerRaw) * currentLsbAmps_ * 25.0F * 1000.0F;
    reading->stateOfChargePercent = -1.0F;
    return ESP_OK;
}
