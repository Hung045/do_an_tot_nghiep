#include "AdaptiveServo.h"

#include <algorithm>
#include <cmath>
#include "BoardConfig.h"
#include "driver/ledc.h"
#include "esp_log.h"

namespace {
constexpr ledc_mode_t kMode = LEDC_LOW_SPEED_MODE;
constexpr ledc_timer_t kTimer = LEDC_TIMER_2;
constexpr ledc_channel_t kChannel = LEDC_CHANNEL_2;
constexpr uint32_t kFrequencyHz = 50;
constexpr uint32_t kPeriodUs = 1000000U / kFrequencyHz;
constexpr uint32_t kCounts = 1U << 16;
static_assert(DEMO_SERVO_MIN_DEG >= 0 && DEMO_SERVO_MIN_DEG < DEMO_SERVO_CENTER_DEG &&
              DEMO_SERVO_CENTER_DEG < DEMO_SERVO_MAX_DEG && DEMO_SERVO_MAX_DEG <= 180);
static_assert(DEMO_SERVO_MIN_PULSE_US >= 500 && DEMO_SERVO_MAX_PULSE_US <= 2500 &&
              DEMO_SERVO_MIN_PULSE_US < DEMO_SERVO_MAX_PULSE_US);
static_assert(DEMO_SERVO_DIRECTION == 1 || DEMO_SERVO_DIRECTION == -1);
static_assert(DEMO_SERVO_RATE_DEG_PER_SEC > 0 && DEMO_SERVO_FILTER_MS > 0 &&
              DEMO_SERVO_GAIN >= 0 && DEMO_SERVO_DEADBAND_DEG >= 0);

uint32_t angleDuty(float degrees)
{
    const float pulse = DEMO_SERVO_MIN_PULSE_US + degrees / 180.0F *
                       (DEMO_SERVO_MAX_PULSE_US - DEMO_SERVO_MIN_PULSE_US);
    return static_cast<uint32_t>(pulse * kCounts / kPeriodUs + 0.5F);
}
}

esp_err_t AdaptiveServo::initialize()
{
    ready_ = tracking_ = false;
    angle_ = target_ = DEMO_SERVO_CENTER_DEG;
    filteredRoll_ = 0;
    ledc_timer_config_t timer = {};
    timer.speed_mode = kMode;
    timer.duty_resolution = LEDC_TIMER_16_BIT;
    timer.timer_num = kTimer;
    timer.freq_hz = kFrequencyHz;
    timer.clk_cfg = LEDC_AUTO_CLK;
    esp_err_t err = ledc_timer_config(&timer);
    if (err != ESP_OK) return err;
    ledc_channel_config_t channel = {};
    channel.gpio_num = DEMO_HEADLIGHT_SERVO_GPIO;
    channel.speed_mode = kMode;
    channel.channel = kChannel;
    channel.intr_type = LEDC_INTR_DISABLE;
    channel.timer_sel = kTimer;
    channel.duty = angleDuty(angle_);
    err = ledc_channel_config(&channel);
    if (err != ESP_OK) return err;
    ready_ = true;
    ESP_LOGI("AdaptiveServo", "SG90 GPIO%d, 50 Hz, center=%.1f deg, limits=%.1f..%.1f deg (command only)",
             DEMO_HEADLIGHT_SERVO_GPIO, static_cast<double>(angle_),
             static_cast<double>(DEMO_SERVO_MIN_DEG), static_cast<double>(DEMO_SERVO_MAX_DEG));
    return ESP_OK;
}

esp_err_t AdaptiveServo::writeAngle(float degrees)
{
    esp_err_t err = ledc_set_duty(kMode, kChannel, angleDuty(degrees));
    if (err == ESP_OK) err = ledc_update_duty(kMode, kChannel);
    if (err != ESP_OK) {
        ledc_stop(kMode, kChannel, 0);
        ready_ = tracking_ = false;
    }
    return err;
}

esp_err_t AdaptiveServo::update(float rollDegrees, bool imuValid, bool fault, uint32_t elapsedMs)
{
    if (!ready_) return ESP_ERR_INVALID_STATE;
    // Cap elapsed time so a delayed control iteration cannot command a large jump.
    const float dtMs = static_cast<float>(std::min<uint32_t>(elapsedMs, 50U));
    tracking_ = imuValid && !fault && std::isfinite(rollDegrees) && std::fabs(rollDegrees) < 60.0F;
    if (tracking_) {
        const float roll = rollDegrees - DEMO_SERVO_ROLL_ZERO_DEG;
        filteredRoll_ += dtMs / (DEMO_SERVO_FILTER_MS + dtMs) * (roll - filteredRoll_);
        const float offset = std::copysign(std::max(0.0F, std::fabs(filteredRoll_) - DEMO_SERVO_DEADBAND_DEG), filteredRoll_);
        target_ = std::clamp(DEMO_SERVO_CENTER_DEG + DEMO_SERVO_DIRECTION * DEMO_SERVO_GAIN * offset,
                             DEMO_SERVO_MIN_DEG, DEMO_SERVO_MAX_DEG);
    } else {
        filteredRoll_ = 0;
        target_ = DEMO_SERVO_CENTER_DEG;
    }
    const float step = DEMO_SERVO_RATE_DEG_PER_SEC * dtMs / 1000.0F;
    const float next = angle_ + std::clamp(target_ - angle_, -step, step);
    if (std::fabs(next - angle_) < 0.001F) return ESP_OK;
    const esp_err_t err = writeAngle(next);
    if (err == ESP_OK) angle_ = next;
    return err;
}
