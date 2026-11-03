#include <cmath>
#include <cstdio>
#include <cstdint>
#include <atomic>

#include "BoardConfig.h"
#include "CrashDetector.h"
#include "DataLogger.h"
#include "GpsTracker.h"
#include "HeadlightController.h"
#include "MotorController.h"
#include "PowerMonitor.h"
#include "ProjectTypes.h"
#include "TelemetryLink.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "sdkconfig.h"

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

enum class ControlState { Locked, Ready, RelaySettling, Running, Fault };

struct TelemetrySnapshot {
    uint32_t uptimeMilliseconds;
    float tiltDegrees;
    int throttleRaw;
    uint32_t throttlePercent;
    float filteredThrottlePercent;
    uint32_t dutyPermille;
    bool relayOn;
    ControlState state;
    bool crashEvent;
};

struct NetworkSnapshot {
    TelemetrySnapshot motor;
    TelemetryRecord record;
};

struct LightSnapshot {
    float ambientLux;
    uint32_t duty;
};

// Khai báo toàn bộ các Module của cả nhóm
CrashDetector crashDetector;
DataLogger dataLogger;
GpsTracker gpsTracker;
MotorController motor;
HeadlightController headlight;
PowerMonitor powerMonitor;
TelemetryLink telemetry;

QueueHandle_t telemetryQueue = nullptr;
QueueHandle_t networkQueue = nullptr;
QueueHandle_t crashQueue = nullptr;
QueueHandle_t lightQueue = nullptr;
std::atomic<bool> safetyFault{false};
bool crashDetectorReadyForTelemetry = false;
bool powerMonitorReadyForTelemetry = false;
bool gpsTrackerReadyForTelemetry = false;
bool dataLoggerReadyForTelemetry = false;

const char *phaseName(ControlState state)
{
    switch (state) {
    case ControlState::Locked: return "LOCKED";
    case ControlState::Ready: return "READY";
    case ControlState::RelaySettling: return "RELAY_WAIT";
    case ControlState::Running: return "RUNNING";
    case ControlState::Fault: return "FAULT_LOCKED";
    }
    return "FAULT_LOCKED";
}

// Only this task calls the network APIs: a socket timeout cannot hold up
// logging, EEPROM, GPS or headlight updates. Queues contain value copies.
void networkTask(void *)
{
    const esp_err_t err = telemetry.initialize(CONFIG_DEMO_MQTT_BROKER_URI);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Network unavailable: %s; local tasks continue", esp_err_to_name(err));
    }
    for (;;) {
        NetworkSnapshot frame = {};
        const bool received = xQueueReceive(networkQueue, &frame, pdMS_TO_TICKS(100)) == pdPASS;

        // A crash has its own queue, so overwriting live samples cannot lose it.
        // Remove it only after MQTT accepts it into its bounded QoS 1 outbox.
        GpsFix eventFix = {};
        if (xQueuePeek(crashQueue, &eventFix, 0) == pdPASS &&
            telemetry.publishEvent(VehicleEvent::CrashDetected, eventFix) == ESP_OK) {
            xQueueReceive(crashQueue, &eventFix, 0);
        }
        if (!received) continue;

        // Event enqueueing can also wait on the MQTT mutex. Refresh afterwards.
        NetworkSnapshot newer = {};
        if (xQueueReceive(networkQueue, &newer, 0) == pdPASS) frame = newer;
        const auto sampleAgeMs = [&]() -> uint32_t {
            return static_cast<uint32_t>(xTaskGetTickCount() * portTICK_PERIOD_MS) -
                   frame.record.uptimeMilliseconds;
        };
        if (sampleAgeMs() > 1500U) continue;

        const TickType_t started = xTaskGetTickCount();
        const auto &snapshot = frame.motor;
        const esp_err_t motorErr = telemetry.publishMotorTelemetry(
            frame.record.uptimeMilliseconds, phaseName(snapshot.state), snapshot.throttleRaw,
            snapshot.throttlePercent, snapshot.filteredThrottlePercent,
            snapshot.dutyPermille, snapshot.relayOn);
        esp_err_t systemErr = ESP_ERR_INVALID_STATE;
        // Do not send the second half of an old frame after a blocked write.
        if (sampleAgeMs() <= 1500U) {
            systemErr = telemetry.publishTelemetry(
                frame.record, crashDetectorReadyForTelemetry, gpsTrackerReadyForTelemetry,
                powerMonitorReadyForTelemetry, dataLoggerReadyForTelemetry);
        }
        const uint32_t elapsedMs = (xTaskGetTickCount() - started) * portTICK_PERIOD_MS;
        if (elapsedMs >= 500U ||
            (motorErr != ESP_OK && motorErr != ESP_ERR_INVALID_STATE) ||
            (systemErr != ESP_OK && systemErr != ESP_ERR_INVALID_STATE)) {
            ESP_LOGW(TAG, "MQTT send=%lu ms | motor=%s system=%s | local tasks continue",
                     static_cast<unsigned long>(elapsedMs),
                     esp_err_to_name(motorErr), esp_err_to_name(systemErr));
        }
    }
}

void headlightTask(void *)
{
    for (;;) {
        LightSnapshot reading = {-1.0F, 0};
        headlight.updateAutoHeadlight(safetyFault.load(), &reading.ambientLux, &reading.duty);
        xQueueOverwrite(lightQueue, &reading);
        // Always yield even when the shared I2C bus is slow.
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void telemetryTask(void *)
{
    PowerReading powerReading = {};
    GpsFix gpsFix = {};
    bool crashQueued = false;
    float ambientLux = -1.0F;
    uint32_t lightDuty = 0;

    for (;;) {
        TelemetrySnapshot snapshot = {};
        const BaseType_t received = xQueueReceive(
            telemetryQueue, &snapshot, pdMS_TO_TICKS(100));
        LightSnapshot light = {};
        if (lightQueue != nullptr && xQueuePeek(lightQueue, &light, 0) == pdPASS) {
            ambientLux = light.ambientLux;
            lightDuty = light.duty;
        }
        if (received != pdPASS) {
            continue;
        }

        if (powerMonitorReadyForTelemetry && powerMonitor.read(&powerReading) != ESP_OK) {
            powerReading = {};
        }

        if (gpsTrackerReadyForTelemetry) {
            const esp_err_t gpsErr = gpsTracker.readFix(&gpsFix);
            if (gpsErr != ESP_OK && gpsErr != ESP_ERR_NOT_FOUND) {
                gpsFix = {};
            }
        }

        TelemetryRecord record = {};
        record.uptimeMilliseconds = snapshot.uptimeMilliseconds;
        record.tiltDegrees = snapshot.tiltDegrees;
        record.throttlePercent = static_cast<uint32_t>(snapshot.filteredThrottlePercent + 0.5F);
        record.busVoltageVolts = powerReading.busVoltageVolts;
        record.currentMilliamps = powerReading.currentMilliamps;
        record.gps = gpsFix;

        // The crash bit remains latched in every control snapshot until reset.
        if (snapshot.crashEvent && !crashQueued && crashQueue != nullptr) {
            crashQueued = xQueueSend(crashQueue, &gpsFix, 0) == pdPASS;
        }
        if (networkQueue != nullptr) {
            const NetworkSnapshot frame = {snapshot, record};
            xQueueOverwrite(networkQueue, &frame);
        }

        bool eepromWriteOk = false;
        if (dataLoggerReadyForTelemetry) {
            eepromWriteOk = (dataLogger.append(record) == ESP_OK);
        }

        const char *phase = phaseName(snapshot.state);
        const char *eepromStatus = !dataLoggerReadyForTelemetry ? "MISSING"
                                                                : (eepromWriteOk ? "OK" : "WRITE_ERROR");

        ESP_LOGI(TAG, "========== VEHICLE | %.1f s | %s ==========",
                 static_cast<double>(record.uptimeMilliseconds / 1000.0F), phase);
        ESP_LOGI(TAG, "MOTOR   throttle=%3lu%% | filtered=%5.1f%% | PWM=%5.1f%% | relay=%s | ADC=%d",
                 static_cast<unsigned long>(snapshot.throttlePercent),
                 static_cast<double>(snapshot.filteredThrottlePercent),
                 static_cast<double>(snapshot.dutyPermille / 10.0F),
                 snapshot.relayOn ? "ON" : "OFF", snapshot.throttleRaw);
        ESP_LOGI(TAG, "SENSORS MPU6050=%s (tilt=%5.1f deg) | BH1750=%s (lux=%6.1f) | light=%s %lu%%",
                 crashDetectorReadyForTelemetry ? "OK" : "MISSING",
                 static_cast<double>(snapshot.tiltDegrees),
                 ambientLux >= 0.0F ? "OK" : "MISSING",
                 static_cast<double>(ambientLux),
                 snapshot.state == ControlState::Fault ? "BLINK" :
                 (lightDuty > 0U ? "ON" : "OFF"),
                 static_cast<unsigned long>(lightDuty));
        if (powerMonitorReadyForTelemetry) {
            ESP_LOGI(TAG, "STORAGE EEPROM=%s | INA226=OK (%.2f V, %.0f mA)",
                     eepromStatus,
                     static_cast<double>(record.busVoltageVolts),
                     static_cast<double>(record.currentMilliamps));
        } else {
            ESP_LOGI(TAG, "STORAGE EEPROM=%s | INA226=MISSING", eepromStatus);
        }

        if (record.gps.valid) {
            ESP_LOGI(TAG, "GPS     FIX=YES | satellites=%lu | lat=%.6f | lon=%.6f | speed=%.2f m/s",
                     static_cast<unsigned long>(record.gps.satellitesInView),
                     record.gps.latitudeDegrees,
                     record.gps.longitudeDegrees,
                     static_cast<double>(record.gps.speedMetersPerSecond));
        } else {
            ESP_LOGI(TAG, "GPS     FIX=NO  | satellites=%lu",
                     static_cast<unsigned long>(record.gps.satellitesInView));
        }

        std::printf("\n");
    }
}

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

    ControlState state = ControlState::Locked;

    bool zeroTiming = false;
    bool relayOn = false;
    bool stopTiming = false;
    uint32_t duty = 0;
    float filteredThrottle = 0.0F;
    uint32_t mpuFailStreak = 0;
    float lastTiltDegrees = 0.0F;

    TickType_t zeroSince = 0;
    TickType_t relaySince = 0;
    TickType_t stopSince = 0;

    auto triggerSafetyFault = [&](const char *operation, esp_err_t cause) {
        if (state == ControlState::Fault) {
            return;
        }
        state = ControlState::Fault;
        safetyFault.store(true);
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
    const bool crashDetectorReady = (err == ESP_OK) && crashDetector.isAvailable();
    if (!crashDetectorReady) {
        ESP_LOGW(TAG, "MPU6050 unavailable: %s", esp_err_to_name(err));
#if CONFIG_DEMO_REQUIRE_MPU6050
        triggerSafetyFault("MPU6050 required but unavailable", ESP_ERR_NOT_FOUND);
#else
        ESP_LOGW(TAG, "BENCH TEST: motor is allowed without MPU6050");
#endif
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

    crashDetectorReadyForTelemetry = crashDetectorReady;
    powerMonitorReadyForTelemetry = powerMonitorReady;
    gpsTrackerReadyForTelemetry = gpsTrackerReady;
    dataLoggerReadyForTelemetry = dataLoggerReady;

    // Publish all initialization flags before starting their consumer tasks.
    networkQueue = xQueueCreate(1, sizeof(NetworkSnapshot));
    crashQueue = xQueueCreate(1, sizeof(GpsFix));
    if (networkQueue == nullptr || crashQueue == nullptr ||
        xTaskCreate(networkTask, "vehicle_network", 6144, nullptr, 2, nullptr) != pdPASS) {
        ESP_LOGE(TAG, "Network task unavailable; local control/logging continue");
        if (networkQueue != nullptr) vQueueDelete(networkQueue);
        if (crashQueue != nullptr) vQueueDelete(crashQueue);
        networkQueue = nullptr;
        crashQueue = nullptr;
    }

    if (headlightReady) {
        lightQueue = xQueueCreate(1, sizeof(LightSnapshot));
        if (lightQueue == nullptr ||
            xTaskCreate(headlightTask, "vehicle_light", 4096, nullptr, 4, nullptr) != pdPASS) {
            if (lightQueue != nullptr) vQueueDelete(lightQueue);
            lightQueue = nullptr;
            triggerSafetyFault("headlight task allocation", ESP_ERR_NO_MEM);
        }
    }

    telemetryQueue = xQueueCreate(1, sizeof(TelemetrySnapshot));
    if (telemetryQueue == nullptr) {
        ESP_LOGE(TAG, "Could not create telemetry queue; local control continues");
    } else if (xTaskCreate(telemetryTask, "vehicle_telemetry", 6144, nullptr, 3, nullptr) != pdPASS) {
        ESP_LOGE(TAG, "Could not create telemetry task; local control continues");
        vQueueDelete(telemetryQueue);
        telemetryQueue = nullptr;
    }

    ESP_LOGI(TAG, "SYSTEM READY: Return throttle <=2%% for 1s to arm; start >=5%%.");

    TickType_t lastWake = xTaskGetTickCount();
    TickType_t lastLog = lastWake;
    bool crashEventLatched = false;

    for (;;) {
        const TickType_t now = xTaskGetTickCount();
        float tiltDegrees = lastTiltDegrees;
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
                if (!crashEventLatched) {
                    crashEventLatched = true;
                }
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

        if (state != ControlState::Fault) {
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
                    if (state != ControlState::Fault) {
                        state = ControlState::Ready;
                    }
                    zeroTiming = false;
                    stopTiming = false;
                }
                if (state == ControlState::Locked) {
                    if (!zeroTiming) {
                        zeroSince = now;
                        zeroTiming = true;
                    }
                    if (static_cast<TickType_t>(now - zeroSince) >= zeroHold) {
                        state = ControlState::Ready;
                        ESP_LOGI(TAG, "ARMED: throttle may now start motor.");
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

                if (state == ControlState::Ready && throttle >= kStartThrottle &&
                    filteredThrottle >= kStartThrottle * 10.0F) {
                    requireOk(motor.setMotorPowerEnabled(true), "relay ON");
                    if (state != ControlState::Fault) {
                        relayOn = true;
                        relaySince = now;
                        state = ControlState::RelaySettling;
                    }
                }
                if (state == ControlState::RelaySettling &&
                    static_cast<TickType_t>(now - relaySince) >= relaySettle) {
                    state = ControlState::Running;
                }
                if (state == ControlState::Running) {
                    const uint32_t nextDuty = rampDuty(duty, static_cast<uint32_t>(filteredThrottle + 0.5F));
                    if (nextDuty != duty) {
                        requireOk(motor.setMotorDutyPermille(nextDuty), "running PWM");
                        duty = nextDuty;
                    }
                }
            }
        }

        // D. Chu kỳ 1 giây: gửi ảnh chụp trạng thái sang task telemetry.
        if (static_cast<TickType_t>(now - lastLog) >= pdMS_TO_TICKS(kTelemetryPeriodMs)) {
            if (telemetryQueue != nullptr) {
                TelemetrySnapshot snapshot = {};
                snapshot.uptimeMilliseconds = static_cast<uint32_t>(now * portTICK_PERIOD_MS);
                snapshot.tiltDegrees = tiltDegrees;
                snapshot.throttleRaw = raw;
                snapshot.throttlePercent = throttle;
                snapshot.filteredThrottlePercent = filteredThrottle / 10.0F;
                snapshot.dutyPermille = duty;
                snapshot.relayOn = relayOn;
                snapshot.state = state;
                snapshot.crashEvent = crashEventLatched;
                xQueueOverwrite(telemetryQueue, &snapshot);
            }
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
