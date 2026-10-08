#pragma once

#include <cstddef>
#include <cstdint>

#include "ProjectTypes.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"

class GpsTracker {
public:
    esp_err_t initialize();
    esp_err_t readFix(GpsFix *fix);

private:
    void consumeByte(uint8_t byte);
    void parseLine();

    char line_[128] = {};
    size_t lineLength_ = 0;
    GpsFix latestFix_ = {};
    TickType_t lastFixAt_ = 0;
    bool hasFix_ = false;
};
