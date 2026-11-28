#include "CrashDetector.h"

#include <cmath>
#include <cstdint>

#include "BoardConfig.h"
#include "esp_log.h"
#include "freertos/task.h"

namespace {

constexpr i2c_port_t kI2cPort = DEMO_MPU_I2C_PORT;
constexpr uint8_t kWhoAmIRegister = 0x75;
constexpr uint8_t kConfigRegister = 0x1A;
constexpr uint8_t kGyroConfigRegister = 0x1B;
constexpr uint8_t kAccelConfigRegister = 0x1C;
constexpr uint8_t kAccelDataRegister = 0x3B;
constexpr uint8_t kPowerManagementRegister = 0x6B;
constexpr float kRadiansToDegrees = 180.0F / 3.14159265F;
constexpr float kCrashTiltDegrees = 60.0F;
constexpr uint32_t kCrashConfirmMs = 200;
static_assert(DEMO_MPU_ROLL_FILTER_MS > 0);

float wrapDegrees(float angle)
{
    return std::remainder(angle, 360.0F);
}

esp_err_t writeRegister(uint8_t registerAddress, uint8_t value)
{
    BoardI2c::LockGuard lock(kI2cPort, pdMS_TO_TICKS(100));
    if (!lock.isLocked()) {
        return ESP_ERR_TIMEOUT;
    }

    const uint8_t data[] = {registerAddress, value};
    return BoardI2c::write(
        kI2cPort,
        DEMO_MPU6050_ADDRESS,
        data,
        sizeof(data),
        pdMS_TO_TICKS(100));
}

} // namespace

esp_err_t CrashDetector::initialize()
{
    available_ = false;
    rollInitialized_ = false;
    tiltTiming_ = false;
    const esp_err_t busErr = BoardI2c::initialize(kI2cPort);
    if (busErr != ESP_OK) {
        available_ = false;
        return busErr;
    }

    uint8_t deviceId = 0;
    esp_err_t err = ESP_FAIL;
    {
        BoardI2c::LockGuard lock(kI2cPort, pdMS_TO_TICKS(100));
        if (!lock.isLocked()) {
            available_ = false;
            return ESP_ERR_TIMEOUT;
        }
        err = BoardI2c::writeRead(
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

    // Explicit ranges: gyro +/-250 deg/s (131 LSB/deg/s), accel +/-2g.
    err = writeRegister(kGyroConfigRegister, 0x00);
    if (err != ESP_OK) return err;
    err = writeRegister(kAccelConfigRegister, 0x00);
    if (err != ESP_OK) return err;
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
        tiltTiming_ = false;
        rollInitialized_ = false;
        *crashDetected = false;
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
    // Coherent accel, temperature (unused), and gyro burst: 0x3B..0x48.
    uint8_t data[14] = {};
    {
        // Dedicated MPU bus: do not wait behind another owner in the control loop.
        BoardI2c::LockGuard lock(kI2cPort, 0);
        if (!lock.isLocked()) {
            return ESP_ERR_TIMEOUT;
        }
        esp_err_t err = BoardI2c::writeRead(
            kI2cPort,
            DEMO_MPU6050_ADDRESS,
            &kAccelDataRegister,
            1,
            data,
            sizeof(data),
            pdMS_TO_TICKS(20));
        if (err != ESP_OK) {
            return err;
        }
    }

    const auto accelX = static_cast<float>(static_cast<int16_t>((static_cast<uint16_t>(data[0]) << 8) | data[1]));
    const auto accelY = static_cast<float>(static_cast<int16_t>((static_cast<uint16_t>(data[2]) << 8) | data[3]));
    const auto accelZ = static_cast<float>(static_cast<int16_t>((static_cast<uint16_t>(data[4]) << 8) | data[5]));

    if (accelX == 0.0F && accelY == 0.0F && accelZ == 0.0F) return ESP_ERR_INVALID_STATE;
    const float accelRoll = std::atan2(DEMO_MPU_ROLL_USE_Y ? accelY : accelX, accelZ) * kRadiansToDegrees;
    const auto gyroX = static_cast<float>(static_cast<int16_t>((static_cast<uint16_t>(data[8]) << 8) | data[9]));
    const auto gyroY = static_cast<float>(static_cast<int16_t>((static_cast<uint16_t>(data[10]) << 8) | data[11]));
    const float rate = (DEMO_MPU_ROLL_USE_Y ? gyroX : -gyroY) / 131.0F - DEMO_MPU_ROLL_GYRO_BIAS_DPS;
    const TickType_t now = xTaskGetTickCount();
    const uint32_t elapsedMs = static_cast<TickType_t>(now - lastRollTick_) * portTICK_PERIOD_MS;
    if (!rollInitialized_ || elapsedMs > 100U) {
        // Never integrate a stale gyro sample across an outage or scheduler gap.
        rollDegrees_ = accelRoll;
        rollInitialized_ = true;
    } else if (elapsedMs > 0U) {
        const float dtMs = static_cast<float>(elapsedMs);
        const float prediction = wrapDegrees(rollDegrees_ + rate * dtMs / 1000.0F);
        const float normG = std::sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ) / 16384.0F;
        // Reject large acceleration disturbances in the lamp filter only.
        // Near-1g acceleration in a sustained turn can still bias this estimate.
        const float correction = normG >= 0.8F && normG <= 1.2F
            ? dtMs / (DEMO_MPU_ROLL_FILTER_MS + dtMs) : 0.0F;
        rollDegrees_ = wrapDegrees(prediction + correction * wrapDegrees(accelRoll - prediction));
    }
    lastRollTick_ = now;

    // Preserve Z's sign: upright=0, on its side=90, upside down=180 degrees.
    *tiltDegrees = std::atan2(std::sqrt((accelX * accelX) + (accelY * accelY)), accelZ) *
                   kRadiansToDegrees;
    return ESP_OK;
}
