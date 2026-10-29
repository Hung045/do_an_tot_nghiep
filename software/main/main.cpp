#include "MotorController.h"
#include "BoardConfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {
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
