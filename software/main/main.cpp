#include <cmath>
#include <cstdint>

#include "BoardConfig.h"
#include "CrashDetector.h"
#include "DataLogger.h"
#include "GpsTracker.h"
#include "ProjectTypes.h"
#include "MotorController.h"
#include "BoardConfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {

constexpr char TAG[] = "my_3_modules_test";
constexpr uint32_t kLoopPeriodMs = 50;          
constexpr uint32_t kTelemetryPeriodMs = 1000;    

CrashDetector crashDetector;
DataLogger dataLogger;
GpsTracker gpsTracker;

void latchSafetyFault(bool *faultLatched, const char *reason)
{
    if (*faultLatched) {
        return;
    }

    *faultLatched = true;
    ESP_LOGE(TAG, "SAFETY LOCK kích hoạt: %s. (Nếu có động cơ, Relay sẽ bị ngắt tại đây!)", reason);
// Duty is expressed in per mille (0..1000), preserving sub-percent steps.
uint32_t rampDuty(uint32_t current, uint32_t target)
{
    if (target > 1000U) target = 1000U;
    if (target > current) return current + (target - current > 10U ? 10U : target - current);
    return current - (current - target > 20U ? 20U : current - target);
}
} // namespace

extern "C" void app_main(void)
{
    esp_err_t err;

    err = crashDetector.initialize();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Crash detector (MPU6050) unavailable: %s", esp_err_to_name(err));
    }

    err = gpsTracker.initialize();
    const bool gpsTrackerReady = (err == ESP_OK);
    if (!gpsTrackerReady) {
        ESP_LOGW(TAG, "NEO-6M unavailable: %s", esp_err_to_name(err));
    }

    err = dataLogger.initialize();
    const bool dataLoggerReady = (err == ESP_OK);
    if (!dataLoggerReady) {
        ESP_LOGW(TAG, "AT24C256 unavailable: %s", esp_err_to_name(err));
    } else {
        TelemetryRecord lastSaved = {};
        if (dataLogger.readLatest(&lastSaved) == ESP_OK) {
            ESP_LOGI(TAG, "[EEPROM Khôi phục] Uptime: %lu ms | Tilt: %.1f deg | GPS: %s (Sat=%lu)",
                     static_cast<unsigned long>(lastSaved.uptimeMilliseconds),
                     static_cast<double>(lastSaved.tiltDegrees),
                     lastSaved.gps.valid ? "VALID" : "NO-FIX",
                     static_cast<unsigned long>(lastSaved.gps.satellitesInView));
        }
    }

    ESP_LOGI(TAG, "=== SẴN SÀNG TEST 3 MODULE (CRASH, LOGGER, GPS) ===");

    bool faultLatched = false;
    TickType_t lastStatusAt = xTaskGetTickCount();
    TickType_t lastTelemetryAt = lastStatusAt;
    GpsFix gpsFix = {};
    constexpr char TAG[] = "motor_test";
    constexpr uint32_t kZeroThrottle = 2; // Dead band for ADC noise near zero.
    constexpr uint32_t kStartThrottle = 5;
    const TickType_t zeroHold = pdMS_TO_TICKS(1000);
    const TickType_t relaySettle = pdMS_TO_TICKS(100);
    const TickType_t stopDebounce = pdMS_TO_TICKS(200);
    MotorController motor;

    // Latch faults until reset. Attempt both outputs even if one command fails.
    auto faultStop = [&](const char *operation, esp_err_t cause) {
        for (;;) {
            const esp_err_t pwmErr = motor.setMotorDutyPercent(0);
            const esp_err_t relayErr = motor.setMotorPowerEnabled(false);
            ESP_LOGE(TAG, "FAULT %s: %s; stop PWM=%s relay=%s; RESET required",
                     operation, esp_err_to_name(cause), esp_err_to_name(pwmErr),
                     esp_err_to_name(relayErr));
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    };
    auto requireOk = [&](esp_err_t result, const char *operation) {
        if (result != ESP_OK) {
            faultStop(operation, result);
        }
    };

    requireOk(motor.initialize(), "initialize");
    requireOk(motor.setMotorDutyPercent(0), "initial PWM zero");
    requireOk(motor.setMotorPowerEnabled(false), "initial relay OFF");

    enum class State { Locked, Ready, RelaySettling, Running };
    State state = State::Locked;
    bool zeroTiming = false;
    bool relayOn = false;
    bool stopTiming = false;
    uint32_t duty = 0; // Per mille, not percent.
    float filteredThrottle = 0.0f;
    TickType_t zeroSince = 0;
    TickType_t relaySince = 0;
    TickType_t stopSince = 0;
    TickType_t lastWake = xTaskGetTickCount();
    TickType_t lastLog = lastWake;

    ESP_LOGW(TAG, "MOTOR TEST: smooth PWM 0..100%%; return throttle <=2%% for 1s to arm; start >=5%%.");
    ESP_LOGI(TAG, "Low throttle must persist for 200 ms to stop; ADC faults stop immediately.");
    ESP_LOGW(TAG, "No current protection or physical relay feedback. Verify first with motor power disconnected.");

    for (;;) {
        int raw = 0;
        uint32_t throttle = 0;
        requireOk(motor.readThrottlePercent(&throttle, &raw), "throttle ADC");
        const TickType_t now = xTaskGetTickCount();
        float tiltDegrees = 0.0F;
        bool crashDetected = false;

        err = crashDetector.update(&tiltDegrees, &crashDetected);
        if (err != ESP_OK) {
            latchSafetyFault(&faultLatched, "MPU6050 rớt kết nối I2C");
        } else if (crashDetected) {
            latchSafetyFault(&faultLatched, "Xe nghiêng vượt ngưỡng 60 độ!");
        }

        if ((now - lastStatusAt) >= pdMS_TO_TICKS(1000)) {
            ESP_LOGI(TAG, "Góc nghiêng: %.1f deg | Trạng thái: %s",
                     static_cast<double>(tiltDegrees),
                     faultLatched ? "LOCKED (Bị ngã/Lỗi)" : "BÌNH THƯỜNG");
            lastStatusAt = now;
        }

        if ((now - lastTelemetryAt) >= pdMS_TO_TICKS(kTelemetryPeriodMs)) {
            if (gpsTrackerReady) {
                err = gpsTracker.readFix(&gpsFix);
                if (err != ESP_OK && err != ESP_ERR_NOT_FOUND) {
                    ESP_LOGW(TAG, "Lỗi đọc GPS: %s", esp_err_to_name(err));
                    gpsFix = {};
                }
            }

            TelemetryRecord record = {};
            record.uptimeMilliseconds = static_cast<uint32_t>(now * portTICK_PERIOD_MS);
            record.tiltDegrees = tiltDegrees;
            record.throttlePercent = 0;       // Giá trị giả lập tay ga
            record.busVoltageVolts = 12.0F;   // Giá trị giả lập điện áp pin 3S
            record.currentMilliamps = 0.0F;   // Giá trị giả lập dòng điện
            record.gps = gpsFix;

            bool eepromWriteOk = false;
            if (dataLoggerReady) {
                err = dataLogger.append(record);
                if (err == ESP_OK) {
                    eepromWriteOk = true;
                } else {
                    ESP_LOGE(TAG, "Lỗi ghi EEPROM: %s", esp_err_to_name(err));
                }
            }

            const char *eepromStatus = !dataLoggerReady ? "Chưa cắm EEPROM"
                                                        : (eepromWriteOk ? "Đã ghi EEPROM" : "Ghi EEPROM lỗi");

            if (record.gps.valid) {
                ESP_LOGI(TAG, "-> %s | GPS BẮT ĐƯỢC (Sat=%lu, Lat=%.6f, Lon=%.6f, Spd=%.1f m/s)",
                         eepromStatus,
                         static_cast<unsigned long>(record.gps.satellitesInView),
                         record.gps.latitudeDegrees,
                         record.gps.longitudeDegrees,
                         static_cast<double>(record.gps.speedMetersPerSecond));
            } else {
                ESP_LOGI(TAG, "-> %s | GPS ĐANG TÌM SÓNG (Sat=%lu)",
                         eepromStatus,
                         static_cast<unsigned long>(record.gps.satellitesInView));
            }

            lastTelemetryAt = now;
        }

        vTaskDelay(pdMS_TO_TICKS(kLoopPeriodMs));
    }
}

} 

extern "C" void app_main(void)
{
    if (xTaskCreate(controlTask, "test_my_modules", 6144, nullptr, 5, nullptr) != pdPASS) {
        ESP_LOGE(TAG, "Không thể tạo Task!");

        if (throttle <= kZeroThrottle) {
            if (!relayOn) filteredThrottle = 0.0f;
            if (relayOn && !stopTiming) {
                stopSince = now;
                stopTiming = true;
            }
            // Hold the previous duty during a short low-input glitch. Do not
            // block sampling or fault handling while confirming a real stop.
            if (relayOn && static_cast<TickType_t>(now - stopSince) >= stopDebounce) {
                requireOk(motor.setMotorDutyPercent(0), "stop PWM");
                duty = 0;
                filteredThrottle = 0.0f;
                requireOk(motor.setMotorPowerEnabled(false), "stop relay");
                relayOn = false;
                state = State::Ready; // Only startup requires the one-second hold.
                zeroTiming = false;
                stopTiming = false;
                ESP_LOGI(TAG, "STOP -> READY: throttle <=2%% for 200 ms; raw=%d throttle=%lu%%",
                         raw, static_cast<unsigned long>(throttle));
            }
            if (state == State::Locked) {
                if (!zeroTiming) {
                    zeroSince = now;
                    zeroTiming = true;
                }
                if (static_cast<TickType_t>(now - zeroSince) >= zeroHold) {
                    state = State::Ready;
                    ESP_LOGI(TAG, "ARMED: throttle may now start motor.");
                }
            }
        } else {
            zeroTiming = false;
            stopTiming = false;
            const int boundedRaw = raw < DEMO_THROTTLE_ADC_MIN ? DEMO_THROTTLE_ADC_MIN :
                                   raw > DEMO_THROTTLE_ADC_MAX ? DEMO_THROTTLE_ADC_MAX : raw;
            const float target = (boundedRaw - DEMO_THROTTLE_ADC_MIN) * 1000.0f /
                                 (DEMO_THROTTLE_ADC_MAX - DEMO_THROTTLE_ADC_MIN);
            filteredThrottle += 0.2f * (target - filteredThrottle);
            if (state == State::Ready && throttle >= kStartThrottle &&
                filteredThrottle >= kStartThrottle * 10.0f) {
                requireOk(motor.setMotorPowerEnabled(true), "relay ON");
                relayOn = true;
                relaySince = now;
                state = State::RelaySettling;
            }
            if (state == State::RelaySettling &&
                static_cast<TickType_t>(now - relaySince) >= relaySettle) {
                state = State::Running;
            }
            if (state == State::Running) {
                const uint32_t nextDuty = rampDuty(duty, static_cast<uint32_t>(filteredThrottle + 0.5f));
                if (nextDuty != duty) {
                    requireOk(motor.setMotorDutyPermille(nextDuty), "running PWM");
                    duty = nextDuty;
                }
            }
        }

        if (static_cast<TickType_t>(now - lastLog) >= pdMS_TO_TICKS(1000)) {
            const char *phase = state == State::Locked ? "LOCKED" :
                                state == State::Ready ? "READY" :
                                state == State::RelaySettling ? "RELAY_WAIT" : "RUNNING";
            ESP_LOGI(TAG, "phase=%s raw=%d throttle=%lu%% relay_cmd=%s filtered=%.1f%% pwm_cmd=%.1f%%",
                     phase, raw, static_cast<unsigned long>(throttle),
                     relayOn ? "ON" : "OFF", static_cast<double>(filteredThrottle / 10.0f),
                     static_cast<double>(duty / 10.0f));
            lastLog = now;
        }
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(20));
    }
}