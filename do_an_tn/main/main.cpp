#include <cmath>
#include <cstdint>

#include "CrashDetector.h"
#include "HeadlightController.h"
#include "MotorController.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {

constexpr char TAG[] = "vehicle_demo";
constexpr uint32_t kLoopPeriodMs = 50;
constexpr uint32_t kThrottleReleaseMs = 1000;
constexpr float kMaximumDimmingTiltDegrees = 60.0F;

CrashDetector crashDetector;
MotorController motorController;
HeadlightController headlightController;

void latchSafetyFault(bool *faultLatched, const char *reason)
{
    if (*faultLatched) {
        return;
    }

    *faultLatched = true;
    esp_err_t err = motorController.setMotorDutyPercent(0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set motor PWM to zero: %s", esp_err_to_name(err));
    }
    err = motorController.setMotorPowerEnabled(false);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to disable motor relay: %s", esp_err_to_name(err));
    }
    err = headlightController.setBrightnessPercent(0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to turn off headlight PWM: %s", esp_err_to_name(err));
    }
    ESP_LOGE(TAG, "SAFETY LOCK: %s. Restart the board after checking the setup.", reason);
}

void controlTask(void *)
{
    esp_err_t err = motorController.initialize();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Motor controller initialization failed: %s", esp_err_to_name(err));
        for (;;) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    err = headlightController.initialize();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Headlight controller initialization failed: %s", esp_err_to_name(err));
        for (;;) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    err = crashDetector.initialize();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Crash detector initialization failed: %s; motor stays off",
                 esp_err_to_name(err));
        for (;;) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    ESP_LOGI(TAG, "Demo ready. Keep throttle at zero for 1 second to arm.");

    bool faultLatched = false;
    bool armed = false;
    uint32_t filteredThrottle = 0;
    uint32_t lightDuty = 0;
    TickType_t throttleReleasedAt = 0;
    TickType_t lastStatusAt = xTaskGetTickCount();

    for (;;) {
        const TickType_t now = xTaskGetTickCount();
        float tiltDegrees = 0.0F;
        bool crashDetected = false;

        err = crashDetector.update(&tiltDegrees, &crashDetected);
        if (err != ESP_OK) {
            latchSafetyFault(&faultLatched, "MPU6050 read failed");
        } else if (crashDetected) {
            latchSafetyFault(&faultLatched, "tilt exceeded 60 degrees");
        }

        uint32_t throttlePercent = 0;
        err = motorController.readThrottlePercent(&throttlePercent);
        if (err != ESP_OK) {
            latchSafetyFault(&faultLatched, "throttle ADC read failed");
            throttlePercent = 0;
        }
        filteredThrottle = ((filteredThrottle * 3U) + throttlePercent) / 4U;

        if (!faultLatched && !armed) {
            if (filteredThrottle <= 5U) {
                if (throttleReleasedAt == 0) {
                    throttleReleasedAt = now;
                } else if ((now - throttleReleasedAt) >= pdMS_TO_TICKS(kThrottleReleaseMs)) {
                    err = motorController.setMotorPowerEnabled(true);
                    if (err == ESP_OK) {
                        armed = true;
                        ESP_LOGI(TAG, "System armed.");
                    } else {
                        latchSafetyFault(&faultLatched, "failed to enable motor relay");
                    }
                }
            } else {
                throttleReleasedAt = 0;
            }
        }

        err = motorController.setMotorDutyPercent(
            (armed && !faultLatched) ? filteredThrottle : 0U);
        if (err != ESP_OK) {
            latchSafetyFault(&faultLatched, "motor PWM update failed");
        }

        const float limitedTilt = (tiltDegrees < kMaximumDimmingTiltDegrees)
                                      ? tiltDegrees
                                      : kMaximumDimmingTiltDegrees;
        const uint32_t targetLightDuty = static_cast<uint32_t>(
            100.0F - ((limitedTilt / kMaximumDimmingTiltDegrees) * 60.0F));
        if (lightDuty < targetLightDuty) {
            lightDuty += (targetLightDuty - lightDuty > 2U)
                             ? 2U
                             : targetLightDuty - lightDuty;
        } else if (lightDuty > targetLightDuty) {
            lightDuty -= (lightDuty - targetLightDuty > 2U)
                             ? 2U
                             : lightDuty - targetLightDuty;
        }

        err = headlightController.setBrightnessPercent(faultLatched ? 0U : lightDuty);
        if (err != ESP_OK) {
            latchSafetyFault(&faultLatched, "headlight PWM update failed");
        }

        if ((now - lastStatusAt) >= pdMS_TO_TICKS(1000)) {
            ESP_LOGI(TAG, "tilt=%.1f deg, throttle=%lu%%, motor=%s, state=%s",
                     static_cast<double>(tiltDegrees),
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
    if (xTaskCreate(controlTask, "vehicle_control", 6144, nullptr, 5, nullptr) != pdPASS) {
        ESP_LOGE(TAG, "Could not create control task; motor outputs were not initialized");
    }
}
