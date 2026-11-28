#include <cstdint>
#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace {
constexpr char kTag[] = "mg90s_test";
constexpr int kServoPin = 14;
constexpr auto kMode = LEDC_LOW_SPEED_MODE;
constexpr auto kTimer = LEDC_TIMER_0;
constexpr auto kChannel = LEDC_CHANNEL_0;
constexpr uint32_t kPeriodUs = 20000;  // 50 Hz
constexpr int kCenter = 90;
constexpr int kMinAngle = 70;
constexpr int kMaxAngle = 110;

// Nominal mapping only: actual shaft angles require physical calibration.
// This test uses only 1389..1611 us, away from the mechanical end stops.
uint32_t pulseForAngle(int angle)
{
    return 1000U + (static_cast<uint32_t>(angle) * 1000U + 90U) / 180U;
}

uint32_t dutyForAngle(int angle)
{
    return (pulseForAngle(angle) * 65536U + kPeriodUs / 2U) / kPeriodUs;
}

bool setAngle(int angle)
{
    if (angle < kMinAngle || angle > kMaxAngle) return false;
    esp_err_t err = ledc_set_duty(kMode, kChannel, dutyForAngle(angle));
    if (err == ESP_OK) err = ledc_update_duty(kMode, kChannel);
    if (err == ESP_OK) return true;
    ledc_stop(kMode, kChannel, 0);
    ESP_LOGE(kTag, "PWM stopped: %s", esp_err_to_name(err));
    return false;
}
}  // namespace

extern "C" void app_main(void)
{
    ledc_timer_config_t timer = {};
    timer.speed_mode = kMode;
    timer.duty_resolution = LEDC_TIMER_16_BIT;
    timer.timer_num = kTimer;
    timer.freq_hz = 50;
    timer.clk_cfg = LEDC_AUTO_CLK;
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    ledc_channel_config_t channel = {};
    channel.gpio_num = kServoPin;
    channel.speed_mode = kMode;
    channel.channel = kChannel;
    channel.intr_type = LEDC_INTR_DISABLE;
    channel.timer_sel = kTimer;
    channel.duty = dutyForAngle(kCenter);
    ESP_ERROR_CHECK(ledc_channel_config(&channel));

    ESP_LOGI(kTag, "MG90S | GPIO%d | 50 Hz | ESP32 + servo only", kServoPin);
    ESP_LOGI(kTag, "cmd is nominal, NOT measured shaft angle. Test without lamp load.");
    ESP_LOGI(kTag, "Disconnect servo power if shaft spins multiple turns or mechanism jams.");

    constexpr int targets[] = {90, 80, 90, 100, 90, 70, 110, 90};
    constexpr unsigned count = sizeof(targets) / sizeof(targets[0]);
    int current = kCenter;
    for (unsigned i = 0; i < count; ++i) {
        const int target = targets[i];
        ESP_LOGI(kTag, "STEP %u/%u | moving to cmd=%d deg", i + 1, count, target);
        // Change the absolute position command by 1 degree every 20 ms.
        // Sending a fixed command repeatedly does not accumulate rotation.
        while (current != target) {
            current += target > current ? 1 : -1;
            if (!setAngle(current)) return;
            vTaskDelay(pdMS_TO_TICKS(20));
        }
        const unsigned holdMs = i == 0 ? 5000U : 3000U;
        ESP_LOGI(kTag, "HOLD | cmd=%d deg | pulse=%lu us | %u ms", target,
                 static_cast<unsigned long>(pulseForAngle(target)), holdMs);
        vTaskDelay(pdMS_TO_TICKS(holdMs));
    }
    ESP_LOGI(kTag, "DONE | holding cmd=90 deg / 1500 us. EN/RESET to repeat.");
    // LEDC hardware keeps the final pulse running after app_main returns.
}
