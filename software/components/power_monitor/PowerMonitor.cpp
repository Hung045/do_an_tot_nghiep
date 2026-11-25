#include "PowerMonitor.h"

#include <cstdint>
#include <limits>

#include "nvs.h"
#include "nvs_flash.h"

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
static_assert(DEMO_BATTERY_EMPTY_MILLIVOLTS < DEMO_BATTERY_NOMINAL_MILLIVOLTS &&
              DEMO_BATTERY_NOMINAL_MILLIVOLTS < DEMO_BATTERY_FULL_MILLIVOLTS,
              "Battery voltage calibration must be increasing");

float estimateBatterySocPercent(float voltage)
{
    constexpr float emptyVoltage = DEMO_BATTERY_EMPTY_MILLIVOLTS / 1000.0F;
    constexpr float nominalVoltage = DEMO_BATTERY_NOMINAL_MILLIVOLTS / 1000.0F;
    constexpr float fullVoltage = DEMO_BATTERY_FULL_MILLIVOLTS / 1000.0F;
    if (voltage <= emptyVoltage) return 0.0F;
    if (voltage >= fullVoltage) return 100.0F;
    if (voltage <= nominalVoltage) {
        return 50.0F * (voltage - emptyVoltage) / (nominalVoltage - emptyVoltage);
    }
    return 50.0F + 50.0F * (voltage - nominalVoltage) / (fullVoltage - nominalVoltage);
}

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
    sohCycleActive_ = false;
    sohFullReached_ = false;
    sohTickInitialized_ = false;
    cycleDischargeMah_ = 0.0F;
    lastSohUpdateTick_ = 0;
    learnedCapacityMah_ = -1.0F;
    stateOfHealthPercent_ = -1.0F;
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

    nvs_handle_t nvsHandle = 0;
    esp_err_t nvsErr = nvs_flash_init();
    if (nvsErr == ESP_OK || nvsErr == ESP_ERR_NVS_NO_FREE_PAGES || nvsErr == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvsErr = nvs_open("battery_health", NVS_READONLY, &nvsHandle);
        if (nvsErr == ESP_OK) {
            uint32_t learnedMah = 0;
            if (nvs_get_u32(nvsHandle, "learned_mah", &learnedMah) == ESP_OK &&
                learnedMah >= 100U) {
                learnedCapacityMah_ = static_cast<float>(learnedMah);
                stateOfHealthPercent_ = 100.0F * learnedCapacityMah_ /
                    static_cast<float>(DEMO_BATTERY_RATED_CAPACITY_MAH);
                stateOfHealthPercent_ = stateOfHealthPercent_ < 0.0F ? 0.0F :
                    (stateOfHealthPercent_ > 100.0F ? 100.0F : stateOfHealthPercent_);
            }
            nvs_close(nvsHandle);
        }
    }

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
    *reading = {unknown, unknown, unknown, -1.0F, -1.0F};
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
    reading->stateOfChargePercent = estimateBatterySocPercent(reading->busVoltageVolts);
    updateSohCycle(reading->busVoltageVolts, rawCurrentMilliamps);
    reading->stateOfHealthPercent = stateOfHealthPercent_;
    return ESP_OK;
}

void PowerMonitor::updateSohCycle(float voltage, float dischargeMilliamps)
{
    constexpr float fullVoltage = DEMO_BATTERY_FULL_MILLIVOLTS / 1000.0F;
    constexpr float cutoffVoltage = DEMO_BATTERY_CUTOFF_MILLIVOLTS / 1000.0F;

    const uint32_t now = static_cast<uint32_t>(xTaskGetTickCount());
    if (!sohTickInitialized_) {
        lastSohUpdateTick_ = now;
        sohTickInitialized_ = true;
    }

    const float measuredDischargeMa =
        DEMO_INA219_DISCHARGE_SIGN * dischargeMilliamps;

    if (measuredDischargeMa < -100.0F) {
        sohCycleActive_ = false;
        sohFullReached_ = false;
        cycleDischargeMah_ = 0.0F;
    }

    if (voltage >= fullVoltage && measuredDischargeMa <= 100.0F) {
        sohFullReached_ = true;
    }

    if (sohFullReached_ && measuredDischargeMa > 100.0F && !sohCycleActive_) {
        sohCycleActive_ = true;
        cycleDischargeMah_ = 0.0F;
    }

    if (sohCycleActive_ && measuredDischargeMa > 100.0F) {
        const uint32_t elapsedTicks = now - lastSohUpdateTick_;
        const float elapsedHours = static_cast<float>(elapsedTicks) *
            static_cast<float>(portTICK_PERIOD_MS) / 3600000.0F;
        cycleDischargeMah_ += measuredDischargeMa * elapsedHours;
    }

    if (sohCycleActive_ && voltage <= cutoffVoltage && measuredDischargeMa > 100.0F) {
        constexpr uint32_t ratedMah = DEMO_BATTERY_RATED_CAPACITY_MAH;
        if (ratedMah > 0U && cycleDischargeMah_ >= static_cast<float>(ratedMah) * 0.5F) {
            learnedCapacityMah_ = cycleDischargeMah_;
            stateOfHealthPercent_ = 100.0F * learnedCapacityMah_ /
                static_cast<float>(ratedMah);
            stateOfHealthPercent_ = stateOfHealthPercent_ < 0.0F ? 0.0F :
                (stateOfHealthPercent_ > 100.0F ? 100.0F : stateOfHealthPercent_);

            nvs_handle_t nvsHandle = 0;
            esp_err_t nvsErr = nvs_open("battery_health", NVS_READWRITE, &nvsHandle);
            if (nvsErr == ESP_OK) {
                const esp_err_t setErr = nvs_set_u32(nvsHandle, "learned_mah",
                    static_cast<uint32_t>(learnedCapacityMah_ + 0.5F));
                const esp_err_t commitErr = setErr == ESP_OK
                    ? nvs_commit(nvsHandle) : setErr;
                nvs_close(nvsHandle);
                if (commitErr != ESP_OK) {
                    ESP_LOGW("PowerMonitor", "SOH NVS save failed: %s", esp_err_to_name(commitErr));
                }
            } else {
                ESP_LOGW("PowerMonitor", "SOH NVS open failed: %s", esp_err_to_name(nvsErr));
            }
            ESP_LOGI("PowerMonitor", "SOH calibration complete: learned=%lu mAh, SOH=%.1f%%",
                     static_cast<unsigned long>(learnedCapacityMah_),
                     static_cast<double>(stateOfHealthPercent_));
        }
        sohCycleActive_ = false;
        sohFullReached_ = false;
        cycleDischargeMah_ = 0.0F;
    }

    lastSohUpdateTick_ = now;
}
