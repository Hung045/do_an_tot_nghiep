#pragma once

#include <cstdint>

#include "esp_err.h"

class HeadlightController {
public:
    esp_err_t initialize();
    esp_err_t setBrightnessPercent(uint32_t percent);
};
