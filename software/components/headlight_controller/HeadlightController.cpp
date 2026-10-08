#include "HeadlightController.h"

#include "BoardConfig.h"
#include "driver/ledc.h"

namespace {

constexpr ledc_mode_t kSpeedMode = LEDC_LOW_SPEED_MODE;
constexpr ledc_timer_t kTimer = LEDC_TIMER_1;
constexpr ledc_channel_t kChannel = LEDC_CHANNEL_1;
constexpr uint32_t kPwmMaximumDuty = (1U << DEMO_PWM_RESOLUTION_BITS) - 1U;

} // namespace

esp_err_t HeadlightController::initialize()
{
    ledc_timer_config_t timer = {};
    timer.speed_mode = kSpeedMode;
    timer.duty_resolution = static_cast<ledc_timer_bit_t>(DEMO_PWM_RESOLUTION_BITS);
    timer.timer_num = kTimer;
    timer.freq_hz = DEMO_PWM_FREQUENCY_HZ;
    timer.clk_cfg = LEDC_AUTO_CLK;

    esp_err_t err = ledc_timer_config(&timer);
    if (err != ESP_OK) {
        return err;
    }

    ledc_channel_config_t channel = {};
    channel.gpio_num = DEMO_HEADLIGHT_PWM_GPIO;
    channel.speed_mode = kSpeedMode;
    channel.channel = kChannel;
    channel.intr_type = LEDC_INTR_DISABLE;
    channel.timer_sel = kTimer;
    channel.duty = 0;
    channel.hpoint = 0;
    return ledc_channel_config(&channel);
}

esp_err_t HeadlightController::setBrightnessPercent(uint32_t percent)
{
    if (percent > 100U) {
        return ESP_ERR_INVALID_ARG;
    }

    const uint32_t duty = (percent * kPwmMaximumDuty) / 100U;
    esp_err_t err = ledc_set_duty(kSpeedMode, kChannel, duty);
    if (err != ESP_OK) {
        return err;
    }
    return ledc_update_duty(kSpeedMode, kChannel);
}
