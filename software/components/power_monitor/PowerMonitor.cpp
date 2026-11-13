#include "PowerMonitor.h"

#include <cstdint>
#include <limits>

#include "BoardConfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {

constexpr i2c_port_t kI2cPort = DEMO_AUX_I2C_PORT;

constexpr uint8_t kConfigurationRegister = 0x00;
constexpr uint8_t kBusVoltageRegister = 0x02;
constexpr uint8_t kPowerRegister = 0x03;
constexpr uint8_t kCurrentRegister = 0x04;
constexpr uint8_t kCalibrationRegister = 0x05;
// 32 V register range (device input maximum is 26 V), +/-320 mV shunt,
// Average 64 samples each for bus and shunt, continuous mode.
// Each conversion takes 34.05 ms (~68.1 ms per bus+shunt cycle).
// Averaging runs inside the sensor, without delaying the control task.
constexpr uint16_t kContinuousShuntAndBusConfig = 0x3F77;
constexpr float kCurrentFilterAlpha =
    static_cast<float>(DEMO_INA219_CURRENT_FILTER_PERCENT) / 100.0F;

esp_err_t writeRegister(uint8_t address, uint16_t value)
{
    BoardI2c::LockGuard lock(kI2cPort, pdMS_TO_TICKS(100));
    if (!lock.isLocked()) {
        return ESP_ERR_TIMEOUT;
    }
    const uint8_t data[] = {
        address,
        static_cast<uint8_t>(value >> 8),
        static_cast<uint8_t>(value & 0xFFU),
    };
    return BoardI2c::write(
        kI2cPort,
        DEMO_INA219_ADDRESS,
        data,
        sizeof(data),
        pdMS_TO_TICKS(100));
}

esp_err_t readRegister(uint8_t address, uint16_t *value)
{
    if (value == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    BoardI2c::LockGuard lock(kI2cPort, pdMS_TO_TICKS(100));
    if (!lock.isLocked()) {
        return ESP_ERR_TIMEOUT;
    }
    uint8_t data[2] = {};
    esp_err_t err = BoardI2c::writeRead(
        kI2cPort,
        DEMO_INA219_ADDRESS,
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
    initialized_ = false;
    hasFilteredCurrent_ = false;
    currentLsbAmps_ = 0.0F;
    filteredCurrentMilliamps_ = 0.0F;
    calibration_ = 0;
    const esp_err_t busErr = BoardI2c::initialize(kI2cPort);
    if (busErr != ESP_OK) return busErr;
    constexpr float shuntResistanceOhms =
        static_cast<float>(DEMO_INA219_SHUNT_MICRO_OHMS) / 1000000.0F;
    constexpr float maximumCurrentAmps =
        static_cast<float>(DEMO_INA219_MAX_CURRENT_MILLIAMPS) / 1000.0F;

    if (shuntResistanceOhms <= 0.0F || maximumCurrentAmps <= 0.0F ||
        kCurrentFilterAlpha <= 0.0F || kCurrentFilterAlpha > 1.0F ||
        // Integer units avoid rounding rejection exactly at the 320 mV limit.
        static_cast<uint64_t>(DEMO_INA219_SHUNT_MICRO_OHMS) *
            DEMO_INA219_MAX_CURRENT_MILLIAMPS > 320000000ULL) {
        return ESP_ERR_INVALID_ARG;
    }

    currentLsbAmps_ = maximumCurrentAmps / 32768.0F;
    const float calibrationValue =
        0.04096F / (currentLsbAmps_ * shuntResistanceOhms);
    if (calibrationValue < 2.0F || calibrationValue > 65535.0F) {
        currentLsbAmps_ = 0.0F;
        return ESP_ERR_INVALID_ARG;
    }

    // Calibration bit 0 is reserved and always reads zero.
    calibration_ = static_cast<uint16_t>(calibrationValue) & 0xFFFEU;
    currentLsbAmps_ = 0.04096F / (calibration_ * shuntResistanceOhms);
    esp_err_t err = writeRegister(kConfigurationRegister, 0x8000U);
    if (err != ESP_OK) {
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(10) + 1);
    // INA219 has no manufacturer/device ID registers. Check reset/config
    // readback instead; this verifies communication, not a unique chip ID.
    uint16_t configuration = 0;
    err = readRegister(kConfigurationRegister, &configuration);
    if (err != ESP_OK) return err;
    if (configuration != 0x399FU) return ESP_ERR_NOT_FOUND;

    err = writeRegister(kConfigurationRegister, kContinuousShuntAndBusConfig);
    if (err != ESP_OK) {
        return err;
    }
    err = writeRegister(kCalibrationRegister, calibration_);
    if (err != ESP_OK) {
        return err;
    }

    uint16_t calibration = 0;
    err = readRegister(kConfigurationRegister, &configuration);
    if (err != ESP_OK) return err;
    err = readRegister(kCalibrationRegister, &calibration);
    if (err != ESP_OK) return err;
    if (configuration != kContinuousShuntAndBusConfig || calibration != calibration_)
        return ESP_ERR_INVALID_STATE;

    initialized_ = true;
    ESP_LOGI("PowerMonitor", "INA219 configured; shunt=%lu uOhm, range=%lu mA; average=64 samples/channel (~68 ms/cycle)",
             static_cast<unsigned long>(DEMO_INA219_SHUNT_MICRO_OHMS),
             static_cast<unsigned long>(DEMO_INA219_MAX_CURRENT_MILLIAMPS));
    return ESP_OK;
}

esp_err_t PowerMonitor::read(PowerReading *reading)
{
    if (reading == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    // Failed reads must not leave old or plausible zero measurements behind.
    const float unknown = std::numeric_limits<float>::quiet_NaN();
    *reading = {unknown, unknown, unknown, -1.0F};
    if (!initialized_ || currentLsbAmps_ <= 0.0F) {
        return ESP_ERR_INVALID_STATE;
    }

    uint16_t busRaw = 0;
    uint16_t currentRaw = 0;
    uint16_t powerRaw = 0;
    uint16_t calibration = 0;

    esp_err_t err = readRegister(kCalibrationRegister, &calibration);
    if (err != ESP_OK) return err;
    if (calibration != calibration_) return ESP_ERR_INVALID_STATE;
    err = readRegister(kBusVoltageRegister, &busRaw);
    if (err != ESP_OK) {
        return err;
    }
    // Reject overflow and incomplete conversions rather than publishing zeros.
    if ((busRaw & 0x01U) != 0 || (busRaw & 0x02U) == 0)
        return ESP_ERR_INVALID_STATE;
    err = readRegister(kCurrentRegister, &currentRaw);
    if (err != ESP_OK) {
        return err;
    }
    err = readRegister(kPowerRegister, &powerRaw);
    if (err != ESP_OK) {
        return err;
    }

    const int16_t signedCurrent = static_cast<int16_t>(currentRaw);
    const float rawCurrentMilliamps =
        static_cast<float>(signedCurrent) * currentLsbAmps_ * 1000.0F;
    // A zero-current sample must stop reporting a decaying old motor current.
    // Start a new filter after zero or after a current-direction reversal.
    if (signedCurrent == 0) {
        filteredCurrentMilliamps_ = 0.0F;
        hasFilteredCurrent_ = false;
    } else if (!hasFilteredCurrent_ ||
               (filteredCurrentMilliamps_ < 0.0F) != (rawCurrentMilliamps < 0.0F)) {
        filteredCurrentMilliamps_ = rawCurrentMilliamps;
        hasFilteredCurrent_ = true;
    } else {
        filteredCurrentMilliamps_ +=
            kCurrentFilterAlpha * (rawCurrentMilliamps - filteredCurrentMilliamps_);
    }
    reading->busVoltageVolts = static_cast<float>(busRaw >> 3) * 0.004F;
    reading->currentMilliamps = filteredCurrentMilliamps_;
    reading->powerMilliwatts =
        static_cast<float>(powerRaw) * currentLsbAmps_ * 20.0F * 1000.0F;
    reading->stateOfChargePercent = -1.0F;
    return ESP_OK;
}
