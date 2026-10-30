#pragma once

#include <cstdint>

#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"

class MotorController {
public:
    esp_err_t initialize();
    esp_err_t readThrottleRaw(int *raw);
    // Optionally return the same ADC sample used to calculate percent.
    esp_err_t readThrottlePercent(uint32_t *percent, int *rawSample = nullptr);
    esp_err_t setMotorDutyPercent(uint32_t percent);
    // Finer command resolution: 0..1000 means 0.0..100.0%.
    esp_err_t setMotorDutyPermille(uint32_t permille);
    esp_err_t setMotorPowerEnabled(bool enabled);

private:
    esp_err_t setPwmDuty(uint32_t channel, uint32_t percent);

    adc_oneshot_unit_handle_t adcHandle_ = nullptr;
};
