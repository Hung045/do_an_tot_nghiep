#include <cmath>
#include <cstdio>
#include <cstdint>

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

// Dự phòng nếu BoardConfig.h chưa định nghĩa ngưỡng ADC của tay ga
#ifndef DEMO_THROTTLE_ADC_MIN
#define DEMO_THROTTLE_ADC_MIN 0
#endif
#ifndef DEMO_THROTTLE_ADC_MAX
#define DEMO_THROTTLE_ADC_MAX 4095
#endif

namespace {

constexpr char TAG[] = "vehicle_system";
constexpr uint32_t kControlPeriodMs = 20;        // Chu kỳ 20ms (50Hz) giúp băm xung động cơ mượt
constexpr uint32_t kTelemetryPeriodMs = 1000;    // Chu kỳ ghi Hộp đen & đọc GPS (1Hz)
constexpr uint32_t kZeroThrottle = 2;            // Ngưỡng chết chống nhiễu ADC tay ga ở mức 0
constexpr uint32_t kStartThrottle = 5;           // Ngưỡng bắt đầu đóng Relay chạy động cơ
constexpr float kMaximumDimmingTiltDegrees = 60.0F;

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

// Khai báo toàn bộ các Module của cả nhóm
CrashDetector crashDetector;
DataLogger dataLogger;
GpsTracker gpsTracker;
MotorController motor;
HeadlightController headlight;
PowerMonitor powerMonitor;
TelemetryLink telemetry;

QueueHandle_t telemetryQueue = nullptr;
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

void telemetryTask(void *)
{
    PowerReading powerReading = {};
    GpsFix gpsFix = {};

    for (;;) {
        TelemetrySnapshot snapshot = {};
        if (xQueueReceive(telemetryQueue, &snapshot, portMAX_DELAY) != pdPASS) {
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

        bool eepromWriteOk = false;
        if (dataLoggerReadyForTelemetry) {
            eepromWriteOk = (dataLogger.append(record) == ESP_OK);
        }

        const char *phase = phaseName(snapshot.state);
        ESP_LOGI(TAG, "phase=%s | tilt=%.1f deg | throttle=%lu%% | relay=%s | pwm=%.1f%%",
                 phase, static_cast<double>(snapshot.tiltDegrees),
                 static_cast<unsigned long>(snapshot.throttlePercent),
                 snapshot.relayOn ? "ON" : "OFF",
                 static_cast<double>(snapshot.dutyPermille / 10.0F));

        const char *eepromStatus = !dataLoggerReadyForTelemetry ? "No-EEPROM"
                                                                : (eepromWriteOk ? "EEPROM-OK" : "EEPROM-ERR");
        if (record.gps.valid) {
            ESP_LOGI(TAG, "telemetry=[%s] %.2fV, %.0fmA | GPS=VALID (Sat=%lu, Lat=%.6f, Lon=%.6f, Spd=%.1fm/s)",
                     eepromStatus,
                     static_cast<double>(record.busVoltageVolts),
                     static_cast<double>(record.currentMilliamps),
                     static_cast<unsigned long>(record.gps.satellitesInView),
                     record.gps.latitudeDegrees,
                     record.gps.longitudeDegrees,
                     static_cast<double>(record.gps.speedMetersPerSecond));
        } else {
            ESP_LOGI(TAG, "telemetry=[%s] %.2fV, %.0fmA | GPS=NO-FIX (Sat=%lu)",
                     eepromStatus,
                     static_cast<double>(record.busVoltageVolts),
                     static_cast<double>(record.currentMilliamps),
                     static_cast<unsigned long>(record.gps.satellitesInView));
        }

        const esp_err_t motorPublishErr = telemetry.publishMotorTelemetry(
            record.uptimeMilliseconds, phase, snapshot.throttleRaw,
            snapshot.throttlePercent, snapshot.filteredThrottlePercent,
            snapshot.dutyPermille, snapshot.relayOn);
        const esp_err_t systemPublishErr = telemetry.publishTelemetry(
            record, crashDetectorReadyForTelemetry, gpsTrackerReadyForTelemetry,
            powerMonitorReadyForTelemetry, dataLoggerReadyForTelemetry);
        if ((motorPublishErr != ESP_OK && motorPublishErr != ESP_ERR_INVALID_STATE) ||
            (systemPublishErr != ESP_OK && systemPublishErr != ESP_ERR_INVALID_STATE)) {
            ESP_LOGW(TAG, "MQTT publish failed: motor=%s system=%s",
                     esp_err_to_name(motorPublishErr),
                     esp_err_to_name(systemPublishErr));
        }
        if (snapshot.crashEvent) {
            const esp_err_t eventErr = telemetry.publishEvent(VehicleEvent::CrashDetected, gpsFix);
            if (eventErr != ESP_OK && eventErr != ESP_ERR_INVALID_STATE) {
                ESP_LOGW(TAG, "Crash event publish failed: %s", esp_err_to_name(eventErr));
            }
        }
        std::printf("\n");
    }
}

// Thuật toán tăng/giảm tốc mềm (Ramp Duty) tính theo phần nghìn (0..1000) của bạn Hùng
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
    uint32_t duty = 0;               // Đơn vị phần nghìn (0..1000)
    float filteredThrottle = 0.0F;   // Đơn vị phần nghìn (0.0..1000.0)
    uint32_t lightDuty = 0;          // Độ sáng đèn pha (0..100%)

    TickType_t zeroSince = 0;
    TickType_t relaySince = 0;
    TickType_t stopSince = 0;

    // Hàm khóa an toàn: Ngắt động cơ & đèn pha ngay lập tức nhưng KHÔNG làm treo CPU
    // để Hộp đen (DataLogger) vẫn tiếp tục ghi lại hiện trường tai nạn.
    auto triggerSafetyFault = [&](const char *operation, esp_err_t cause) {
        if (state == ControlState::Fault) {
            return;
        }
        state = ControlState::Fault;
        duty = 0;
        filteredThrottle = 0.0F;
        relayOn = false;

        const esp_err_t pwmErr = motor.setMotorDutyPercent(0);
        const esp_err_t relayErr = motor.setMotorPowerEnabled(false);
        headlight.setBrightnessPercent(0);

        ESP_LOGE(TAG, "SAFETY FAULT [%s]: %s | Stop PWM=%s, Relay=%s | RESET required!",
                 operation, esp_err_to_name(cause),
                 esp_err_to_name(pwmErr), esp_err_to_name(relayErr));
    };

    auto requireOk = [&](esp_err_t result, const char *operation) {
        if (result != ESP_OK) {
            triggerSafetyFault(operation, result);
        }
    };

    // 1. Khởi tạo Khối Chấp hành
    requireOk(motor.initialize(), "motor init");
    requireOk(motor.setMotorDutyPercent(0), "initial PWM zero");
    requireOk(motor.setMotorPowerEnabled(false), "initial relay OFF");

    const esp_err_t networkErr = telemetry.initialize(CONFIG_DEMO_MQTT_BROKER_URI);
    if (networkErr != ESP_OK) {
        ESP_LOGW(TAG, "Network unavailable: %s; local control continues",
                 esp_err_to_name(networkErr));
    }

    esp_err_t err = headlight.initialize();
    const bool headlightReady = (err == ESP_OK);
    if (!headlightReady) {
        ESP_LOGW(TAG, "Headlight init failed: %s", esp_err_to_name(err));
    }

    // 2. Khởi tạo 3 Module Cảm biến & Hộp đen của bạn (+ INA226)
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
    bool pendingCrashEvent = false;

    for (;;) {
        const TickType_t now = xTaskGetTickCount();
        float tiltDegrees = 0.0F;
        bool crashDetected = false;

        // A. Kiểm tra góc nghiêng & phát hiện ngã xe (CrashDetector)
        if (crashDetectorReady) {
            err = crashDetector.update(&tiltDegrees, &crashDetected);
            if (err != ESP_OK) {
                triggerSafetyFault("MPU6050 read failed", err);
            } else if (crashDetected && state != ControlState::Fault) {
                triggerSafetyFault("Crash detected (Tilt >= 60 deg)", ESP_ERR_INVALID_STATE);
                pendingCrashEvent = true;
            }
        }

        // B. Đọc tay ga & Điều khiển Động cơ theo máy trạng thái của bạn Hùng
        int raw = 0;
        uint32_t throttle = 0;
        err = motor.readThrottlePercent(&throttle, &raw);
        if (err != ESP_OK) {
            triggerSafetyFault("throttle ADC", err);
            throttle = 0;
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
                    ESP_LOGI(TAG, "STOP -> READY: throttle <=2%% for 200 ms; raw=%d throttle=%lu%%",
                             raw, static_cast<unsigned long>(throttle));
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

        // C. Điều khiển Đèn pha thích ứng theo góc nghiêng
        if (headlightReady) {
            const float limitedTilt = (tiltDegrees < kMaximumDimmingTiltDegrees)
                                          ? tiltDegrees
                                          : kMaximumDimmingTiltDegrees;
            const uint32_t targetLightDuty = static_cast<uint32_t>(
                100.0F - ((limitedTilt / kMaximumDimmingTiltDegrees) * 60.0F));
            if (lightDuty < targetLightDuty) {
                lightDuty += (targetLightDuty - lightDuty > 2U) ? 2U : (targetLightDuty - lightDuty);
            } else if (lightDuty > targetLightDuty) {
                lightDuty -= (lightDuty - targetLightDuty > 2U) ? 2U : (lightDuty - targetLightDuty);
            }
            headlight.setBrightnessPercent(state == ControlState::Fault ? 0U : lightDuty);
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
                snapshot.crashEvent = pendingCrashEvent;
                if (xQueueOverwrite(telemetryQueue, &snapshot) == pdPASS) {
                    pendingCrashEvent = false;
                }
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
