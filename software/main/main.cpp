#include <cmath>
#include <cstdint>

#include "BoardConfig.h"
#include "CrashDetector.h"
#include "DataLogger.h"
#include "GpsTracker.h"
#include "HeadlightController.h"
#include "MotorController.h"
#include "PowerMonitor.h"
#include "ProjectTypes.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifndef DEMO_THROTTLE_ADC_MIN
#define DEMO_THROTTLE_ADC_MIN 0
#endif
#ifndef DEMO_THROTTLE_ADC_MAX
#define DEMO_THROTTLE_ADC_MAX 4095
#endif

namespace {

constexpr char TAG[] = "vehicle_system";
constexpr uint32_t kControlPeriodMs = 20;
constexpr uint32_t kTelemetryPeriodMs = 1000;
constexpr uint32_t kZeroThrottle = 2;
constexpr uint32_t kStartThrottle = 5;
constexpr bool kInvertThrottleDirection = true;

CrashDetector crashDetector;
DataLogger dataLogger;
GpsTracker gpsTracker;
MotorController motor;
HeadlightController headlight;
PowerMonitor powerMonitor;

uint32_t rampDuty(uint32_t current, uint32_t target)
{
    if (target > 1000U) target = 1000U;
    if (target > current) return current + (target - current > 10U ? 10U : target - current);
    return current - (current - target > 20U ? 20U : current - target);
}

void controlTask(void *)
{
    const TickType_t zeroHold = pdMS_TO_TICKS(1000);
    const TickType_t relaySettle = pdMS_TO_TICKS(100);
    const TickType_t stopDebounce = pdMS_TO_TICKS(200);

    enum class State { Locked, Ready, RelaySettling, Running, Fault };
    State state = State::Locked;

    bool zeroTiming = false;
    bool relayOn = false;
    bool stopTiming = false;
    uint32_t duty = 0;
    float filteredThrottle = 0.0F;
    uint32_t lightDuty = 0;
    uint32_t mpuFailStreak = 0;
    float lastTiltDegrees = 0.0F;

    TickType_t zeroSince = 0;
    TickType_t relaySince = 0;
    TickType_t stopSince = 0;

    auto triggerSafetyFault = [&](const char *operation, esp_err_t cause) {
        if (state == State::Fault) {
            return;
        }
        state = State::Fault;
        duty = 0;
        filteredThrottle = 0.0F;
        relayOn = false;

        const esp_err_t pwmErr = motor.setMotorDutyPercent(0);
        const esp_err_t relayErr = motor.setMotorPowerEnabled(false);

        ESP_LOGE(TAG, "SAFETY FAULT [%s]: %s | Stop PWM=%s, Relay=%s | RESET required!",
                 operation, esp_err_to_name(cause),
                 esp_err_to_name(pwmErr), esp_err_to_name(relayErr));
    };

    auto requireOk = [&](esp_err_t result, const char *operation) {
        if (result != ESP_OK) {
            triggerSafetyFault(operation, result);
        }
    };

    BoardI2c::getMutex();

    requireOk(motor.initialize(), "motor init");
    requireOk(motor.setMotorDutyPercent(0), "initial PWM zero");
    requireOk(motor.setMotorPowerEnabled(false), "initial relay OFF");

    esp_err_t err = headlight.initialize();
    const bool headlightReady = (err == ESP_OK);
    if (!headlightReady) {
        ESP_LOGW(TAG, "Headlight init failed: %s", esp_err_to_name(err));
    }

    err = crashDetector.initialize();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "MPU6050 unavailable: %s", esp_err_to_name(err));
    }

    err = powerMonitor.initialize();
    const bool powerMonitorReady = (err == ESP_OK);
    if (!powerMonitorReady) {
        ESP_LOGW(TAG, "INA226 unavailable: %s", esp_err_to_name(err));
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
            ESP_LOGI(TAG, "[EEPROM Restored] Uptime: %lu ms | Tilt: %.1f deg | GPS: %s (Sat=%lu)",
                     static_cast<unsigned long>(lastSaved.uptimeMilliseconds),
                     static_cast<double>(lastSaved.tiltDegrees),
                     lastSaved.gps.valid ? "VALID" : "NO-FIX",
                     static_cast<unsigned long>(lastSaved.gps.satellitesInView));
        }
    }

    ESP_LOGI(TAG, "SYSTEM READY: Return throttle <=2%% for 1s to ARM; >=5%% to run.");

    TickType_t lastWake = xTaskGetTickCount();
    TickType_t lastLog = lastWake;
    PowerReading powerReading = {};
    GpsFix gpsFix = {};

    for (;;) {
        const TickType_t now = xTaskGetTickCount();
        float tiltDegrees = lastTiltDegrees;
        float ambientLux = -1.0F;
        bool crashDetected = false;

        err = crashDetector.update(&tiltDegrees, &crashDetected);
        if (err != ESP_OK) {
            mpuFailStreak++;
            if (mpuFailStreak >= 10U) { // Cho phep bo qua nhieu ngan han tren bus I2C (<200ms)
                triggerSafetyFault("MPU6050 read failed", err);
            }
        } else {
            mpuFailStreak = 0;
            lastTiltDegrees = tiltDegrees;
            if (crashDetected) {
                triggerSafetyFault("Crash detected (Tilt >= 60 deg)", ESP_ERR_INVALID_STATE);
            }
        }

        int raw = 0;
        uint32_t throttle = 0;
        err = motor.readThrottlePercent(&throttle, &raw);
        if (err != ESP_OK) {
            triggerSafetyFault("throttle ADC", err);
            throttle = 0;
        } else if (kInvertThrottleDirection) {
            throttle = (throttle <= 100U) ? (100U - throttle) : 0U;
            int bounded = raw < DEMO_THROTTLE_ADC_MIN ? DEMO_THROTTLE_ADC_MIN :
                          raw > DEMO_THROTTLE_ADC_MAX ? DEMO_THROTTLE_ADC_MAX : raw;
            raw = DEMO_THROTTLE_ADC_MAX - (bounded - DEMO_THROTTLE_ADC_MIN);
        }

        if (state != State::Fault) {
            if (throttle <= kZeroThrottle) {
                if (!relayOn) filteredThrottle = 0.0F;
                if (relayOn && !stopTiming) {
                    stopSince = now;
                    stopTiming = true;
                }
                if (relayOn && static_cast<TickType_t>(now - stopSince) >= stopDebounce) {
                    requireOk(motor.setMotorDutyPercent(0), "stop PWM");
                    duty = 0;
                    filteredThrottle = 0.0F;
                    requireOk(motor.setMotorPowerEnabled(false), "stop relay");
                    relayOn = false;
                    if (state != State::Fault) {
                        state = State::Ready;
                    }
                    zeroTiming = false;
                    stopTiming = false;
                }
                if (state == State::Locked) {
                    if (!zeroTiming) {
                        zeroSince = now;
                        zeroTiming = true;
                    }
                    if (static_cast<TickType_t>(now - zeroSince) >= zeroHold) {
                        state = State::Ready;
                        ESP_LOGI(TAG, "ARMED: Motor ready.");
                    }
                }
            } else {
                zeroTiming = false;
                stopTiming = false;
                const int boundedRaw = raw < DEMO_THROTTLE_ADC_MIN ? DEMO_THROTTLE_ADC_MIN :
                                       raw > DEMO_THROTTLE_ADC_MAX ? DEMO_THROTTLE_ADC_MAX : raw;
                const float target = (boundedRaw - DEMO_THROTTLE_ADC_MIN) * 1000.0F /
                                     (DEMO_THROTTLE_ADC_MAX - DEMO_THROTTLE_ADC_MIN);
                filteredThrottle += 0.2F * (target - filteredThrottle);

                if (state == State::Ready && throttle >= kStartThrottle &&
                    filteredThrottle >= kStartThrottle * 10.0F) {
                    requireOk(motor.setMotorPowerEnabled(true), "relay ON");
                    if (state != State::Fault) {
                        relayOn = true;
                        relaySince = now;
                        state = State::RelaySettling;
                    }
                }
                if (state == State::RelaySettling &&
                    static_cast<TickType_t>(now - relaySince) >= relaySettle) {
                    state = State::Running;
                }
                if (state == State::Running) {
                    const uint32_t nextDuty = rampDuty(duty, static_cast<uint32_t>(filteredThrottle + 0.5F));
                    if (nextDuty != duty) {
                        requireOk(motor.setMotorDutyPermille(nextDuty), "running PWM");
                        duty = nextDuty;
                    }
                }
            }
        }

        if (headlightReady) {
            headlight.updateAutoHeadlight(state == State::Fault, &ambientLux, &lightDuty);
        }

        if (static_cast<TickType_t>(now - lastLog) >= pdMS_TO_TICKS(kTelemetryPeriodMs)) {
            if (powerMonitorReady) {
                if (powerMonitor.read(&powerReading) != ESP_OK) {
                    powerReading = {};
                }
            }

            if (gpsTrackerReady) {
                err = gpsTracker.readFix(&gpsFix);
                if (err != ESP_OK && err != ESP_ERR_NOT_FOUND) {
                    gpsFix = {};
                }
            }

            TelemetryRecord record = {};
            record.uptimeMilliseconds = static_cast<uint32_t>(now * portTICK_PERIOD_MS);
            record.tiltDegrees = tiltDegrees;
            record.throttlePercent = static_cast<uint32_t>((filteredThrottle / 10.0F) + 0.5F);
            record.busVoltageVolts = powerReading.busVoltageVolts;
            record.currentMilliamps = powerReading.currentMilliamps;
            record.gps = gpsFix;

            bool eepromWriteOk = false;
            if (dataLoggerReady) {
                eepromWriteOk = (dataLogger.append(record) == ESP_OK);
            }

            const char *phase = state == State::Locked ? "LOCKED" :
                                state == State::Ready ? "READY" :
                                state == State::RelaySettling ? "RELAY_WAIT" :
                                state == State::Running ? "RUNNING" : "FAULT_LOCKED";

            const char *eepromStatus = !dataLoggerReady ? "No-EEPROM"
                                                        : (eepromWriteOk ? "EEPROM-OK" : "EEPROM-ERR");

            const char *lightStatus = (state == State::Fault) ? "HAZARD_BLINK" :
                                      (lightDuty > 0U)        ? "ON (Dark)" : "OFF (Bright)";

            ESP_LOGI(TAG, "------------------------------------------------------------");
            ESP_LOGI(TAG, "[1. MOTOR]     Phase: %s | Throttle: %lu%% (ADC=%d) | Relay: %s | PWM: %.1f%%",
                     phase, static_cast<unsigned long>(throttle), raw,
                     relayOn ? "ON" : "OFF", static_cast<double>(duty / 10.0F));

            ESP_LOGI(TAG, "[2. MPU6050]   Tilt: %.1f deg | Crash Status: %s",
                     static_cast<double>(tiltDegrees),
                     (state == State::Fault) ? "CRASHED / FAULT!" : "NORMAL");

            ESP_LOGI(TAG, "[3. BH1750]    Lux: %.1f lux | Headlight: %s (Duty=%lu%%)",
                     static_cast<double>(ambientLux), lightStatus, static_cast<unsigned long>(lightDuty));

            ESP_LOGI(TAG, "[4. INA226]    Bus Voltage: %.2f V | Current: %.0f mA",
                     static_cast<double>(record.busVoltageVolts),
                     static_cast<double>(record.currentMilliamps));

            if (record.gps.valid) {
                ESP_LOGI(TAG, "[5. NEO-6M]    Fix: VALID (Sat=%lu) | Lat: %.6f | Lon: %.6f | Speed: %.1f m/s",
                         static_cast<unsigned long>(record.gps.satellitesInView),
                         record.gps.latitudeDegrees, record.gps.longitudeDegrees,
                         static_cast<double>(record.gps.speedMetersPerSecond));
            } else {
                ESP_LOGI(TAG, "[5. NEO-6M]    Fix: NO-FIX | Satellites in view: %lu",
                         static_cast<unsigned long>(record.gps.satellitesInView));
            }

            ESP_LOGI(TAG, "[6. AT24C256]  Status: %s | Uptime: %lu ms",
                     eepromStatus, static_cast<unsigned long>(record.uptimeMilliseconds));
            ESP_LOGI(TAG, "============================================================\n");

            lastLog = now;
        }

        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(kControlPeriodMs));
    }
}

} // namespace

extern "C" void app_main(void)
{
    if (xTaskCreate(controlTask, "vehicle_control", 6144, nullptr, 5, nullptr) != pdPASS) {
        ESP_LOGE(TAG, "Could not create vehicle control task!");
    }
}
