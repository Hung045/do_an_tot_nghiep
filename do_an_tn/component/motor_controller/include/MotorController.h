#pragma once

#include <cstdint>

#include "esp_err.h"

class MotorController {
public:
    esp_err_t initialize();
    esp_err_t readThrottlePercent(uint32_t *percent);
    esp_err_t setMotorDutyPercent(uint32_t percent);
    esp_err_t setHeadlightDutyPercent(uint32_t percent);
    esp_err_t setMotorPowerEnabled(bool enabled);

private:
    esp_err_t setPwmDuty(uint32_t channel, uint32_t percent);
};