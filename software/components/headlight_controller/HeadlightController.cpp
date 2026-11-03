#include "HeadlightController.h"

#include "BoardConfig.h"
#include "driver/i2c.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifndef DEMO_I2C_PORT
#define DEMO_I2C_PORT I2C_NUM_0
#endif

namespace {

constexpr char TAG[] = "HeadlightController";
constexpr ledc_mode_t kSpeedMode = LEDC_LOW_SPEED_MODE;
constexpr ledc_timer_t kTimer = LEDC_TIMER_1;
constexpr ledc_channel_t kChannel = LEDC_CHANNEL_1;
constexpr uint32_t kPwmMaximumDuty = (1U << DEMO_PWM_RESOLUTION_BITS) - 1U;

constexpr uint8_t kBh1750PowerOn = 0x01;
constexpr uint8_t kBh1750ContHighResMode = 0x10;

constexpr float kDarkTurnOnLux = 30.0F;
constexpr float kBrightTurnOffLux = 60.0F;
constexpr uint32_t kFaultBlinkHalfPeriodTicks = 12U;

uint8_t sActiveBh1750Addr = DEMO_BH1750_ADDRESS;
float sLastValidLux = -1.0F;
TickType_t sLastLuxReadTicks = 0;

bool tryInitBh1750At(uint8_t addr)
{
    uint8_t cmd = kBh1750PowerOn;
    if (i2c_master_write_to_device(DEMO_I2C_PORT, addr, &cmd, 1, pdMS_TO_TICKS(50)) != ESP_OK) {
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
    cmd = kBh1750ContHighResMode;
    return (i2c_master_write_to_device(DEMO_I2C_PORT, addr, &cmd, 1, pdMS_TO_TICKS(50)) == ESP_OK);
}

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
        ESP_LOGE(TAG, "LEDC timer config failed!");
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

    err = ledc_channel_config(&channel);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "LEDC channel config failed!");
        return err;
    }

    BoardI2c::getMutex();

    {
        BoardI2c::LockGuard lock;
        if (lock.isLocked()) {
            if (tryInitBh1750At(0x23)) {
                sActiveBh1750Addr = 0x23;
                bh1750Ready_ = true;
            } else if (tryInitBh1750At(0x5C)) {
                sActiveBh1750Addr = 0x5C;
                bh1750Ready_ = true;
            }
        }
    }

    if (bh1750Ready_) {
        ESP_LOGI(TAG, "BH1750 initialized at 0x%02X", sActiveBh1750Addr);
        vTaskDelay(pdMS_TO_TICKS(180));
        readAmbientLux(&sLastValidLux);
        sLastLuxReadTicks = xTaskGetTickCount();
    } else {
        ESP_LOGW(TAG, "BH1750 not detected at 0x23 or 0x5C; headlight defaults to ON");
        lightOnState_ = true;
    }

    return ESP_OK;
}

esp_err_t HeadlightController::setBrightnessPercent(uint32_t percent)
{
    if (percent > 100U) return ESP_ERR_INVALID_ARG;
    const uint32_t duty = (percent * kPwmMaximumDuty) / 100U;
    esp_err_t err = ledc_set_duty(kSpeedMode, kChannel, duty);
    if (err != ESP_OK) return err;
    return ledc_update_duty(kSpeedMode, kChannel);
}

esp_err_t HeadlightController::readAmbientLux(float *luxOut)
{
    if (luxOut == nullptr) return ESP_ERR_INVALID_ARG;
    if (!bh1750Ready_) {
        *luxOut = -1.0F;
        return ESP_ERR_INVALID_STATE;
    }

    BoardI2c::LockGuard lock(pdMS_TO_TICKS(50));
    if (!lock.isLocked()) return ESP_ERR_TIMEOUT;

    uint8_t rawData[2] = {0, 0};
    esp_err_t err = i2c_master_read_from_device(
        DEMO_I2C_PORT, sActiveBh1750Addr, rawData, sizeof(rawData), pdMS_TO_TICKS(30));

    if (err == ESP_OK) {
        const uint16_t rawCounts = (static_cast<uint16_t>(rawData[0]) << 8) | rawData[1];
        *luxOut = static_cast<float>(rawCounts) / 1.2F;
    }
    return err;
}

esp_err_t HeadlightController::updateAutoHeadlight(bool isFault, float *luxOut, uint32_t *dutyOut)
{
    const TickType_t now = xTaskGetTickCount();

    if (bh1750Ready_) {
        if (static_cast<TickType_t>(now - sLastLuxReadTicks) >= pdMS_TO_TICKS(200)) {
            float sample = -1.0F;
            if (readAmbientLux(&sample) == ESP_OK) {
                sLastValidLux = sample;
            }
            sLastLuxReadTicks = now;
        }
    }

    if (luxOut != nullptr) {
        *luxOut = sLastValidLux;
    }

    if (isFault) {
        blinkTickCount_++;
        if (blinkTickCount_ >= kFaultBlinkHalfPeriodTicks) {
            blinkTickCount_ = 0;
            blinkPhaseOn_ = !blinkPhaseOn_;
        }
        currentDuty_ = blinkPhaseOn_ ? 100U : 0U;
        if (dutyOut != nullptr) *dutyOut = currentDuty_;
        return setBrightnessPercent(currentDuty_);
    }

    blinkTickCount_ = 0;
    blinkPhaseOn_ = false;

    if (bh1750Ready_ && sLastValidLux >= 0.0F) {
        if (sLastValidLux < kDarkTurnOnLux) {
            lightOnState_ = true;
        } else if (sLastValidLux > kBrightTurnOffLux) {
            lightOnState_ = false;
        }
    } else {
        lightOnState_ = true;
    }

    const uint32_t targetDuty = lightOnState_ ? 100U : 0U;

    if (currentDuty_ < targetDuty) {
        currentDuty_ += (targetDuty - currentDuty_ > 5U) ? 5U : (targetDuty - currentDuty_);
    } else if (currentDuty_ > targetDuty) {
        currentDuty_ -= (currentDuty_ - targetDuty > 5U) ? 5U : (currentDuty_ - targetDuty);
    }

    if (dutyOut != nullptr) *dutyOut = currentDuty_;
    return setBrightnessPercent(currentDuty_);
}
