#include <cmath>
#include <cstdint>

#include "MotorController.h"
#include "MotorControllerConfig.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {

constexpr char TAG[] = "vehicle_demo";
constexpr i2c_port_t kI2cPort = I2C_NUM_0;
constexpr uint8_t kMpu6050WhoAmIRegister = 0x75;
constexpr uint8_t kMpu6050AccelDataRegister = 0x3B;
constexpr uint8_t kMpu6050PowerRegister = 0x6B;
constexpr uint32_t kLoopPeriodMs = 50;
constexpr uint32_t kFallConfirmMs = 200;
constexpr uint32_t kThrottleReleaseMs = 1000;
constexpr float kFallAngleDegrees = 60.0F;
constexpr float kMpu6050AccelCountsPerG = 16384.0F;

MotorController motorController;
bool faultLatched = false;
bool armed = false;

esp_err_t writeMpuRegister(uint8_t registerAddress, uint8_t value)
{
    const uint8_t data[] = {registerAddress, value};
    return i2c_master_write_to_device(
        kI2cPort,
        DEMO_MPU6050_ADDRESS,
        data,
        sizeof(data),
        pdMS_TO_TICKS(100));
}

esp_err_t initializeMpu6050()
{
    i2c_config_t config = {};
    config.mode = I2C_MODE_MASTER;
    config.sda_io_num = DEMO_I2C_SDA_GPIO;
    config.scl_io_num = DEMO_I2C_SCL_GPIO;
    config.sda_pullup_en = GPIO_PULLUP_ENABLE;
    config.scl_pullup_en = GPIO_PULLUP_ENABLE;
    config.master.clk_speed = 100000;

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
        &kMpu6050WhoAmIRegister,
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

    return writeMpuRegister(kMpu6050PowerRegister, 0x00);
}

esp_err_t readTiltAngle(float *angleDegrees)
{
    uint8_t data[6] = {};
    esp_err_t err = i2c_master_write_read_device(
        kI2cPort,
        DEMO_MPU6050_ADDRESS,
        &kMpu6050AccelDataRegister,
        1,
        data,
        sizeof(data),
        pdMS_TO_TICKS(100));
    if (err != ESP_OK) {
        return err;
    }

    const int16_t accelX = static_cast<int16_t>((data[0] << 8) | data[1]);
    const int16_t accelY = static_cast<int16_t>((data[2] << 8) | data[3]);
    const int16_t accelZ = static_cast<int16_t>((data[4] << 8) | data[5]);
    const float xG = static_cast<float>(accelX) / kMpu6050AccelCountsPerG;
    const float yG = static_cast<float>(accelY) / kMpu6050AccelCountsPerG;
    const float zG = static_cast<float>(accelZ) / kMpu6050AccelCountsPerG;

    *angleDegrees = std::atan2(
        std::sqrt((xG * xG) + (yG * yG)),
        std::fabs(zG)) * (180.0F / 3.14159265F);
    return ESP_OK;
}

void latchFault(const char *reason)
{
    if (faultLatched) {
        return;
    }

    faultLatched = true;
    motorController.setMotorDutyPercent(0);
    motorController.setMotorPowerEnabled(false);
    ESP_LOGE(TAG, "SAFETY LOCK: %s. Restart the board after checking the setup.", reason);
}

void appTask(void *)
{
    esp_err_t err = motorController.initialize();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Motor controller initialization failed: %s", esp_err_to_name(err));
        for (;;) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    err = initializeMpu6050();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "MPU6050 initialization failed: %s; motor remains disabled",
                 esp_err_to_name(err));
        for (;;) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    ESP_LOGI(TAG, "Demo ready. Keep throttle at zero for 1 second to arm.");

    uint32_t filteredThrottle = 0;
    uint32_t lightDuty = 0;
    TickType_t throttleReleasedAt = 0;
    TickType_t tiltDetectedAt = 0;
    bool tiltTiming = false;
    TickType_t lastStatusAt = xTaskGetTickCount();

    for (;;) {
        const TickType_t now = xTaskGetTickCount();
        float angleDegrees = 0.0F;
        uint32_t throttlePercent = 0;

        err = readTiltAngle(&angleDegrees);
        if (err != ESP_OK) {
            latchFault("MPU6050 read failed");
        }

        err = motorController.readThrottlePercent(&throttlePercent);
        if (err != ESP_OK) {
            latchFault("throttle ADC read failed");
            throttlePercent = 0;
        }

        filteredThrottle = ((filteredThrottle * 3U) + throttlePercent) / 4U;

        if (!faultLatched && !armed) {
            if (filteredThrottle <= 5U) {
                if (throttleReleasedAt == 0) {
                    throttleReleasedAt = now;
                } else if ((now - throttleReleasedAt) >= pdMS_TO_TICKS(kThrottleReleaseMs)) {
                    armed = true;
                    motorController.setMotorPowerEnabled(true);
                    ESP_LOGI(TAG, "System armed.");
                }
            } else {
                throttleReleasedAt = 0;
            }
        }

        if (!faultLatched && err == ESP_OK) {
            if (angleDegrees >= kFallAngleDegrees) {
                if (!tiltTiming) {
                    tiltTiming = true;
                    tiltDetectedAt = now;
                } else if ((now - tiltDetectedAt) >= pdMS_TO_TICKS(kFallConfirmMs)) {
                    latchFault("tilt exceeded 60 degrees");
                }
            } else {
                tiltTiming = false;
            }
        }

        const uint32_t motorDuty = (armed && !faultLatched) ? filteredThrottle : 0U;
        motorController.setMotorDutyPercent(motorDuty);

        const float limitedAngle = (angleDegrees > kFallAngleDegrees)
                                       ? kFallAngleDegrees
                                       : angleDegrees;
        const uint32_t targetLightDuty = static_cast<uint32_t>(
            100.0F - ((limitedAngle / kFallAngleDegrees) * 60.0F));
        if (lightDuty < targetLightDuty) {
            lightDuty += (targetLightDuty - lightDuty > 2U)
                             ? 2U
                             : targetLightDuty - lightDuty;
        } else if (lightDuty > targetLightDuty) {
            lightDuty -= (lightDuty - targetLightDuty > 2U)
                             ? 2U
                             : lightDuty - targetLightDuty;
        }
        motorController.setHeadlightDutyPercent(faultLatched ? 0U : lightDuty);

        if ((now - lastStatusAt) >= pdMS_TO_TICKS(1000)) {
            ESP_LOGI(TAG, "tilt=%.1f deg, throttle=%lu%%, motor=%s, state=%s",
                     static_cast<double>(angleDegrees),
                     static_cast<unsigned long>(filteredThrottle),
                     (armed && !faultLatched) ? "enabled" : "off",
                     faultLatched ? "LOCKED" : (armed ? "ARMED" : "WAIT_THROTTLE_ZERO"));
            lastStatusAt = now;
        }

        vTaskDelay(pdMS_TO_TICKS(kLoopPeriodMs));
    }
}

} // namespace

extern "C" void app_main(void)
{
    xTaskCreate(appTask, "vehicle_demo", 4096, nullptr, 5, nullptr);
}
