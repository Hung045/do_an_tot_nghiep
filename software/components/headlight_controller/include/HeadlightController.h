#pragma once

#include <cstdint>
#include "esp_err.h"

class HeadlightController {
public:
    esp_err_t initialize();
    esp_err_t setBrightnessPercent(uint32_t percent);
    esp_err_t readAmbientLux(float *luxOut);
    esp_err_t updateAutoHeadlight(bool isFault, float *luxOut = nullptr, uint32_t *dutyOut = nullptr);

private:
    bool bh1750Ready_ = false;
    bool lightOnState_ = false;
    uint32_t currentDuty_ = 0;
    uint32_t blinkTickCount_ = 0;
    bool blinkPhaseOn_ = false;
};