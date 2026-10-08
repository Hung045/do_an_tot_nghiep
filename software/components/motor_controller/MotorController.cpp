#include "MotorController.h"

#include "BoardConfig.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

namespace {

constexpr uint32_t kPwmMaximumDuty = (1U << DEMO_PWM_RESOLUTION_BITS) - 1U;

} // namespace

esp_err_t MotorController::initialize()
{
    esp_err_t err = gpio_set_direction(DEMO_RELAY_GPIO, GPIO_MODE_OUTPUT);
    if (err != ESP_OK) {
        return err;
    }
    err = setMotorPowerEnabled(false);
    if (err != ESP_OK) {
        return err;
    }

    adc_oneshot_unit_init_cfg_t adcUnitConfig = {};
    adcUnitConfig.unit_id = ADC_UNIT_1;
    adcUnitConfig.ulp_mode = ADC_ULP_MODE_DISABLE;
    err = adc_oneshot_new_unit(&adcUnitConfig, &adcHandle_);
    if (err != ESP_OK) {
        return err;
    }

    adc_oneshot_chan_cfg_t throttleConfig = {};
    throttleConfig.atten = ADC_ATTEN_DB_12;
    throttleConfig.bitwidth = ADC_BITWIDTH_DEFAULT;
    err = adc_oneshot_config_channel(adcHandle_, DEMO_THROTTLE_ADC_CHANNEL, &throttleConfig);
    if (err != ESP_OK) {
        return err;
    }

    ledc_timer_config_t timer = {};
    timer.speed_mode = LEDC_LOW_SPEED_MODE;
    timer.duty_resolution = static_cast<ledc_timer_bit_t>(DEMO_PWM_RESOLUTION_BITS);
    timer.timer_num = LEDC_TIMER_0;
    timer.freq_hz = DEMO_PWM_FREQUENCY_HZ;
    timer.clk_cfg = LEDC_AUTO_CLK;
    err = ledc_timer_config(&timer);
    if (err != ESP_OK) {
        return err;
    }

    ledc_channel_config_t motorChannel = {};
    motorChannel.gpio_num = DEMO_MOTOR_PWM_GPIO;
    motorChannel.speed_mode = LEDC_LOW_SPEED_MODE;
    motorChannel.channel = LEDC_CHANNEL_0;
    motorChannel.intr_type = LEDC_INTR_DISABLE;
    motorChannel.timer_sel = LEDC_TIMER_0;
    motorChannel.duty = 0;
    motorChannel.hpoint = 0;
    return ledc_channel_config(&motorChannel);
}

esp_err_t MotorController::readThrottlePercent(uint32_t *percent)
{
    if (percent == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    if (adcHandle_ == nullptr) {
        return ESP_ERR_INVALID_STATE;
    }

    int raw = 0;
    const esp_err_t err = adc_oneshot_read(adcHandle_, DEMO_THROTTLE_ADC_CHANNEL, &raw);
    if (err != ESP_OK) {
        return err;
    }

    *percent = (static_cast<uint32_t>(raw) * 100U) / 4095U;
    return ESP_OK;
}

esp_err_t MotorController::setPwmDuty(uint32_t channel, uint32_t percent)
{
    if (percent > 100U) {
        return ESP_ERR_INVALID_ARG;
    }

    const uint32_t duty = (percent * kPwmMaximumDuty) / 100U;
    esp_err_t err = ledc_set_duty(LEDC_LOW_SPEED_MODE, static_cast<ledc_channel_t>(channel), duty);
    if (err != ESP_OK) {
        return err;
    }
    return ledc_update_duty(LEDC_LOW_SPEED_MODE, static_cast<ledc_channel_t>(channel));
}

esp_err_t MotorController::setMotorDutyPercent(uint32_t percent)
{
    return setPwmDuty(LEDC_CHANNEL_0, percent);
}

esp_err_t MotorController::setMotorPowerEnabled(bool enabled)
{
    const uint32_t activeLevel = DEMO_RELAY_ACTIVE_LEVEL;
    const uint32_t inactiveLevel = activeLevel == 0U ? 1U : 0U;
    return gpio_set_level(DEMO_RELAY_GPIO, enabled ? activeLevel : inactiveLevel);
}