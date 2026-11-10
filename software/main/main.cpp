#include <cmath>
#include <cstdio>
#include <cstdint>
#include <atomic>
#include <limits>

#include "BoardConfig.h"
#include "CrashDetector.h"
#include "MpuHealth.h"
#include "DataLogger.h"
#include "GpsTracker.h"
#include "HeadlightController.h"
#include "AdaptiveServo.h"
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
constexpr uint32_t kTelemetryPeriodMs = 200;
constexpr uint32_t kStoragePeriodMs = 1000;
constexpr uint32_t kMonitorPeriodMs = 1000;
constexpr uint32_t kZeroThrottle = 2;
constexpr uint32_t kStartThrottle = 5;
constexpr bool kInvertThrottleDirection = true;

enum class ControlState { Locked, Ready, RelaySettling, Running, SensorWait, Fault };

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
    float rollDegrees;
    float servoDegrees;
    float servoTargetDegrees;
    bool rollValid;
    bool servoReady;
    bool servoTracking;
    const char *faultReason;
};

struct NetworkSnapshot {
    TelemetrySnapshot motor;
    TelemetryRecord record;
    GpsDiagnostics gpsDiagnostics;
    bool powerValid;
    bool eepromWriteOk;
    LightingTelemetry lighting;
};

struct LightSnapshot {
    float ambientLux;
    uint32_t duty;
    bool outputValid;
    bool faultBlink;
    TickType_t sampledAt;
};

struct StorageSnapshot {
    PowerReading power;
    bool powerValid;
    bool writeOk;
    TickType_t sampledAt;
};

// Khai báo toàn bộ các Module của cả nhóm
CrashDetector crashDetector;
DataLogger dataLogger;
GpsTracker gpsTracker;
MotorController motor;
HeadlightController headlight;
AdaptiveServo headlightServo;
PowerMonitor powerMonitor;
TelemetryLink telemetry;

QueueHandle_t telemetryQueue = nullptr;
QueueHandle_t networkQueue = nullptr;
QueueHandle_t crashQueue = nullptr;
QueueHandle_t lightQueue = nullptr;
QueueHandle_t storageQueue = nullptr;
QueueHandle_t storageStatusQueue = nullptr;
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
    case ControlState::SensorWait: return "SENSOR_WAIT";
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
            snapshot.dutyPermille, snapshot.relayOn, snapshot.faultReason);
        esp_err_t systemErr = ESP_ERR_INVALID_STATE;
        // Do not send the second half of an old frame after a blocked write.
        if (sampleAgeMs() <= 1500U) {
            systemErr = telemetry.publishTelemetry(
                frame.record, crashDetectorReadyForTelemetry, gpsTrackerReadyForTelemetry,
                powerMonitorReadyForTelemetry, frame.powerValid,
                dataLoggerReadyForTelemetry, frame.gpsDiagnostics,
                snapshot.rollValid, frame.eepromWriteOk, frame.lighting);
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
    esp_err_t previousError = ESP_OK;
    for (;;) {
        LightSnapshot reading = {};
        reading.ambientLux = -1.0F;
        reading.faultBlink = safetyFault.load();
        const esp_err_t lightErr = headlight.updateAutoHeadlight(
            reading.faultBlink, &reading.ambientLux, &reading.duty);
        reading.outputValid = lightErr == ESP_OK;
        reading.sampledAt = xTaskGetTickCount();
        if (lightErr != ESP_OK && lightErr != previousError) {
            ESP_LOGW(TAG, "Headlight PWM error: %s", esp_err_to_name(lightErr));
        }
        previousError = lightErr;
        xQueueOverwrite(lightQueue, &reading);
        // Always yield even when the shared I2C bus is slow.
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Slow auxiliary I2C work never blocks assembling live motor/GPS frames.
void storageTask(void *)
{
    for (;;) {
        TelemetryRecord record = {};
        if (xQueueReceive(storageQueue, &record, portMAX_DELAY) != pdPASS) continue;
        StorageSnapshot status = {};
        const esp_err_t err = powerMonitorReadyForTelemetry
            ? powerMonitor.read(&status.power) : ESP_ERR_INVALID_STATE;
        status.powerValid = err == ESP_OK && std::isfinite(status.power.busVoltageVolts) &&
                            std::isfinite(status.power.currentMilliamps);
        status.sampledAt = xTaskGetTickCount();
        record.busVoltageVolts = status.powerValid ? status.power.busVoltageVolts :
            std::numeric_limits<float>::quiet_NaN();
        record.currentMilliamps = status.powerValid ? status.power.currentMilliamps :
            std::numeric_limits<float>::quiet_NaN();
        status.writeOk = dataLoggerReadyForTelemetry && dataLogger.append(record) == ESP_OK;
        xQueueOverwrite(storageStatusQueue, &status);
        // No catch-up writes: consume only the latest pending sample next time.
        vTaskDelay(pdMS_TO_TICKS(kStoragePeriodMs));
    }
}

void telemetryTask(void *)
{
    PowerReading powerReading = {};
    GpsFix gpsFix = {};
    bool crashQueued = false;
    float ambientLux = -1.0F;
    uint32_t lightDuty = 0;
    TickType_t lastMonitor = xTaskGetTickCount() - pdMS_TO_TICKS(kMonitorPeriodMs);

    for (;;) {
        TelemetrySnapshot snapshot = {};
        const BaseType_t received = xQueueReceive(
            telemetryQueue, &snapshot, pdMS_TO_TICKS(100));
        LightSnapshot light = {};
        if (lightQueue != nullptr && xQueuePeek(lightQueue, &light, 0) == pdPASS) {
            ambientLux = light.ambientLux;
            lightDuty = light.duty;
        }
        // Drain UART between snapshots (normally every 100 ms).
        if (gpsTrackerReadyForTelemetry) {
            const esp_err_t gpsErr = gpsTracker.readFix(&gpsFix);
            if (gpsErr != ESP_OK && gpsErr != ESP_ERR_NOT_FOUND) {
                gpsFix = {};
            }
        }

        if (received != pdPASS) continue;
        const GpsDiagnostics gpsDiagnostics = gpsTracker.diagnostics();
        StorageSnapshot storage = {};
        const bool storageFresh = storageStatusQueue != nullptr &&
            xQueuePeek(storageStatusQueue, &storage, 0) == pdPASS &&
            static_cast<TickType_t>(xTaskGetTickCount() - storage.sampledAt) <= pdMS_TO_TICKS(2500);
        powerReading = storage.power;
        const bool powerValid = storageFresh && storage.powerValid;

        TelemetryRecord record = {};
        record.uptimeMilliseconds = snapshot.uptimeMilliseconds;
        record.tiltDegrees = snapshot.rollValid ? snapshot.tiltDegrees : std::numeric_limits<float>::quiet_NaN();
        record.crashDetected = snapshot.crashEvent;
        record.throttlePercent = static_cast<uint32_t>(snapshot.filteredThrottlePercent + 0.5F);
        // EEPROM preserves NaN as an unknown measurement without changing its layout.
        record.busVoltageVolts = powerValid ? powerReading.busVoltageVolts : std::numeric_limits<float>::quiet_NaN();
        record.currentMilliamps = powerValid ? powerReading.currentMilliamps : std::numeric_limits<float>::quiet_NaN();
        record.gps = gpsFix;

        // The crash bit remains latched in every control snapshot until reset.
        if (snapshot.crashEvent && !crashQueued && crashQueue != nullptr) {
            crashQueued = xQueueSend(crashQueue, &gpsFix, 0) == pdPASS;
        }
        const bool eepromWriteOk = storageFresh && storage.writeOk;
        if (storageQueue != nullptr) xQueueOverwrite(storageQueue, &record);
        if (networkQueue != nullptr) {
            LightingTelemetry lighting = {};
            const bool lightFresh = lightQueue != nullptr &&
                static_cast<TickType_t>(xTaskGetTickCount() - light.sampledAt) <= pdMS_TO_TICKS(500);
            lighting.outputValid = lightFresh && light.outputValid;
            lighting.ambientLux = lightFresh ? ambientLux : -1.0F;
            lighting.dutyPercent = lightDuty;
            lighting.faultBlink = light.faultBlink;
            lighting.servoReady = snapshot.servoReady;
            lighting.servoTracking = snapshot.servoTracking;
            lighting.rollValid = snapshot.rollValid;
            lighting.rollDegrees = snapshot.rollDegrees;
            lighting.servoDegrees = snapshot.servoDegrees;
            lighting.servoTargetDegrees = snapshot.servoTargetDegrees;
            const NetworkSnapshot frame = {snapshot, record, gpsDiagnostics, powerValid, eepromWriteOk, lighting};
            xQueueOverwrite(networkQueue, &frame);
        }

        const TickType_t monitorNow = xTaskGetTickCount();
        if (static_cast<TickType_t>(monitorNow - lastMonitor) < pdMS_TO_TICKS(kMonitorPeriodMs)) continue;
        lastMonitor = monitorNow;
        const char *phase = phaseName(snapshot.state);
        const char *eepromStatus = !dataLoggerReadyForTelemetry ? "MISSING"
                                                                : (eepromWriteOk ? "OK" : "WRITE_ERROR");

        ESP_LOGI(TAG, "========== VEHICLE | %.1f s | %s ==========",
                 static_cast<double>(record.uptimeMilliseconds / 1000.0F), phase);
        if (snapshot.state == ControlState::Fault)
            ESP_LOGW(TAG, "LOCK REASON: %s | MPU reads paused; fix cause and RESET", snapshot.faultReason);
        ESP_LOGI(TAG, "MOTOR   throttle=%lu%% | PWM=%.1f%% | relay=%s",
                 static_cast<unsigned long>(snapshot.throttlePercent),
                 static_cast<double>(snapshot.dutyPermille / 10.0F),
                 snapshot.relayOn ? "ON" : "OFF");
        if (crashDetectorReadyForTelemetry && snapshot.rollValid && snapshot.state != ControlState::Fault) {
            ESP_LOGI(TAG, "MPU6050 OK | tilt=%.1f deg", static_cast<double>(record.tiltDegrees));
        } else {
            ESP_LOGI(TAG, "MPU6050 %s", !crashDetectorReadyForTelemetry ? "MISSING" :
                     snapshot.state == ControlState::Fault ? "PAUSED_FAULT" : "RECOVERING");
        }
        if (std::isfinite(ambientLux) && ambientLux >= 0.0F) {
            ESP_LOGI(TAG, "BH1750  OK | lux=%.1f", static_cast<double>(ambientLux));
        } else {
            ESP_LOGI(TAG, "BH1750  UNAVAILABLE");
        }
        ESP_LOGI(TAG, "LIGHT   %s | duty=%lu%%",
                 snapshot.state == ControlState::Fault ? "BLINK" :
                 (lightDuty > 0U ? "ON" : "OFF"),
                 static_cast<unsigned long>(lightDuty));
        ESP_LOGI(TAG, "EEPROM  %s", eepromStatus);
        if (powerValid) {
            ESP_LOGI(TAG, "INA226  OK | %.2f V | %.0f mA",
                     static_cast<double>(record.busVoltageVolts),
                     static_cast<double>(record.currentMilliamps));
        } else if (powerMonitorReadyForTelemetry) {
            ESP_LOGW(TAG, "INA226  DATA_UNAVAILABLE | read failed, pending or stale");
        } else {
            ESP_LOGI(TAG, "INA226  MISSING");
        }

        ESP_LOGI(TAG, "SERVO   %s | command=%.1f deg",
                 !snapshot.servoReady ? "UNAVAILABLE" : snapshot.servoTracking ? "TRACKING" : "CENTERING",
                 static_cast<double>(snapshot.servoDegrees));

        ESP_LOGD(TAG, "GPS UART status=%s | RX=%lu B | NMEA_OK=%lu | rejected=%lu | rx_age=%ld ms | nmea_age=%ld ms",
                 gpsStatusName(gpsDiagnostics.status),
                 static_cast<unsigned long>(gpsDiagnostics.receivedBytes),
                 static_cast<unsigned long>(gpsDiagnostics.validSentences),
                 static_cast<unsigned long>(gpsDiagnostics.rejectedSentences),
                 static_cast<long>(gpsDiagnostics.rxAgeMs),
                 static_cast<long>(gpsDiagnostics.nmeaAgeMs));
        if (record.gps.valid) {
            ESP_LOGI(TAG, "GPS     FIX | sat=%lu | lat=%.6f | lon=%.6f",
                     static_cast<unsigned long>(record.gps.satellitesInView),
                     record.gps.latitudeDegrees,
                     record.gps.longitudeDegrees);
        } else {
            ESP_LOGI(TAG, "GPS     %s | sat=%lu", gpsStatusName(gpsDiagnostics.status),
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
    const TickType_t relaySettle = pdMS_TO_TICKS(100);

    ControlState state = ControlState::Locked;

    bool relayOn = false;
    uint32_t duty = 0;
    float filteredThrottle = 0.0F;
    float lastTiltDegrees = 0.0F;
    MpuHealth mpuHealth;
    const char *faultReason = "";

    TickType_t relaySince = 0;

    auto triggerSafetyFault = [&](const char *operation, esp_err_t cause) {
        if (state == ControlState::Fault) {
            return;
        }
        state = ControlState::Fault;
        faultReason = operation;
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

    requireOk(motor.initialize(), "motor init");
    requireOk(motor.setMotorDutyPercent(0), "initial PWM zero");
    requireOk(motor.setMotorPowerEnabled(false), "initial relay OFF");

    const esp_err_t servoInitErr = headlightServo.initialize();
    if (servoInitErr != ESP_OK) {
        ESP_LOGW(TAG, "Adaptive servo unavailable: %s", esp_err_to_name(servoInitErr));
    }

    esp_err_t err = headlight.initialize();
    const bool headlightReady = (err == ESP_OK);
    if (!headlightReady) {
        ESP_LOGW(TAG, "Headlight init failed: %s", esp_err_to_name(err));
    }

    err = crashDetector.initialize();
    const bool crashDetectorReady = (err == ESP_OK) && crashDetector.isAvailable();
    if (!crashDetectorReady) {
        const esp_err_t missingCause = err == ESP_OK ? ESP_ERR_NOT_FOUND : err;
        ESP_LOGW(TAG, "MPU6050 unavailable: %s", esp_err_to_name(missingCause));
#if CONFIG_DEMO_REQUIRE_MPU6050
        triggerSafetyFault("MPU6050 required but unavailable", missingCause);
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
            ESP_LOGI(TAG, "[EEPROM Restored] crash_latched=%s", lastSaved.crashDetected ? "YES" : "NO");
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

    storageQueue = xQueueCreate(1, sizeof(TelemetryRecord));
    storageStatusQueue = xQueueCreate(1, sizeof(StorageSnapshot));
    if (storageQueue == nullptr || storageStatusQueue == nullptr ||
        xTaskCreate(storageTask, "vehicle_storage", 4096, nullptr, 2, nullptr) != pdPASS) {
        if (storageQueue != nullptr) vQueueDelete(storageQueue);
        if (storageStatusQueue != nullptr) vQueueDelete(storageStatusQueue);
        storageQueue = nullptr;
        storageStatusQueue = nullptr;
        ESP_LOGE(TAG, "Storage task unavailable; live telemetry continues without power readings/logging");
    }

    telemetryQueue = xQueueCreate(1, sizeof(TelemetrySnapshot));
    if (telemetryQueue == nullptr) {
        ESP_LOGE(TAG, "Could not create telemetry queue; local control continues");
    } else if (xTaskCreate(telemetryTask, "vehicle_telemetry", 6144, nullptr, 3, nullptr) != pdPASS) {
        ESP_LOGE(TAG, "Could not create telemetry task; local control continues");
        vQueueDelete(telemetryQueue);
        telemetryQueue = nullptr;
    }

    ESP_LOGI(TAG, "CONTROL START: phase=%s; throttle <=2%% to arm immediately; start >=5%%.", phaseName(state));

    TickType_t lastWake = xTaskGetTickCount();
    TickType_t lastLog = lastWake;
    TickType_t lastServoUpdate = lastWake;
    bool crashEventLatched = false;
    ControlState lastReportedState = state;

    for (;;) {
        const TickType_t now = xTaskGetTickCount();
        float tiltDegrees = lastTiltDegrees;
        bool crashDetected = false;

        // A latched fault pauses MPU traffic until reset instead of hammering a failed bus.
        err = state == ControlState::Fault ? ESP_ERR_INVALID_STATE :
            crashDetector.update(&tiltDegrees, &crashDetected);
        bool rollValid = err == ESP_OK && crashDetector.isAvailable();
        const float rollDegrees = rollValid ? crashDetector.rollDegrees() : 0.0F;
        if (crashDetectorReady && state != ControlState::Fault) {
            const auto health = mpuHealth.update(rollValid,
                static_cast<uint32_t>(xTaskGetTickCount() * portTICK_PERIOD_MS));
            if (health != MpuHealth::State::Healthy) {
                if (state != ControlState::SensorWait) {
                    requireOk(motor.setMotorDutyPercent(0), "MPU hold PWM");
                    requireOk(motor.setMotorPowerEnabled(false), "MPU hold relay");
                    duty = 0;
                    relayOn = false;
                    filteredThrottle = 0;
                    if (state != ControlState::Fault) state = ControlState::SensorWait;
                    ESP_LOGW(TAG, "MPU RECOVERING: motor stopped; require stable samples within 200 ms");
                }
                if (health == MpuHealth::State::Fault)
                    triggerSafetyFault("MPU6050 recovery deadline exceeded", ESP_ERR_TIMEOUT);
                // Even a single good sample is not enough to resume servo tracking.
                rollValid = false;
            } else if (state == ControlState::SensorWait) {
                state = ControlState::Locked;
                ESP_LOGI(TAG, "MPU RECOVERED: return throttle <=2%% to re-arm immediately");
            }
        }
        if (err == ESP_OK) {
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

        if (state != ControlState::Fault && state != ControlState::SensorWait) {
            if (throttle <= kZeroThrottle) {
                // Stop on the first valid low-throttle sample, bypassing the ramp.
                filteredThrottle = 0.0F;
                if (relayOn || duty != 0) {
                    requireOk(motor.setMotorDutyPercent(0), "stop PWM");
                    duty = 0;
                    filteredThrottle = 0.0F;
                    requireOk(motor.setMotorPowerEnabled(false), "stop relay");
                    relayOn = false;
                    if (state != ControlState::Fault) {
                        state = ControlState::Ready;
                        ESP_LOGI(TAG, "STOP: throttle <=2%% | PWM=0%% | relay command=OFF");
                    }
                }
                if (state == ControlState::Locked) {
                    state = ControlState::Ready;
                    ESP_LOGI(TAG, "ARMED: zero throttle detected; throttle may now start motor.");
                }
            } else {
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
                        if (state != ControlState::Fault) duty = nextDuty;
                    }
                }
            }
        }

        // Retry safe outputs if an earlier stop API failed. Never re-enable here.
        if (state == ControlState::Fault) {
            motor.setMotorDutyPercent(0);
            motor.setMotorPowerEnabled(false);
            duty = 0;
            relayOn = false;
            filteredThrottle = 0;
            rollValid = false;
        }

        // Servo shares the control task's MPU sample, never reads I2C1 or waits on MQTT.
        const TickType_t servoNow = xTaskGetTickCount();
        if (headlightServo.isReady()) {
            const esp_err_t servoErr = headlightServo.update(
                rollDegrees, rollValid, safetyFault.load(),
                static_cast<uint32_t>(static_cast<TickType_t>(servoNow - lastServoUpdate) * portTICK_PERIOD_MS));
            if (servoErr != ESP_OK) ESP_LOGW(TAG, "Servo PWM stopped after error: %s", esp_err_to_name(servoErr));
        }
        lastServoUpdate = servoNow;

        // Live state every 200 ms, or immediately on a control-state transition.
        if (state != lastReportedState ||
            static_cast<TickType_t>(now - lastLog) >= pdMS_TO_TICKS(kTelemetryPeriodMs)) {
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
                snapshot.rollDegrees = rollDegrees;
                snapshot.rollValid = rollValid;
                snapshot.servoDegrees = headlightServo.angleDegrees();
                snapshot.servoTargetDegrees = headlightServo.targetDegrees();
                snapshot.servoReady = headlightServo.isReady();
                snapshot.servoTracking = headlightServo.tracking();
                snapshot.faultReason = faultReason;
                xQueueOverwrite(telemetryQueue, &snapshot);
            }
            lastLog = now;
            lastReportedState = state;
        }

        // Skip missed periods; do not issue burst I2C reads to catch up after a timeout.
        const TickType_t period = pdMS_TO_TICKS(kControlPeriodMs);
        if (static_cast<TickType_t>(xTaskGetTickCount() - lastWake) >= period)
            lastWake = xTaskGetTickCount();
        vTaskDelayUntil(&lastWake, period);
    }
}

} // namespace

extern "C" void app_main(void)
{
#if CONFIG_FREERTOS_UNICORE
    constexpr BaseType_t controlCore = 0;
#else
    constexpr BaseType_t controlCore = 1;
#endif
    if (xTaskCreatePinnedToCore(controlTask, "vehicle_control", 6144, nullptr, 5, nullptr, controlCore) != pdPASS) {
        ESP_LOGE(TAG, "Could not create vehicle control task!");
    }
}
