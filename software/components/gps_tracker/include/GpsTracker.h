#pragma once

#include <cstddef>
#include <cstdint>

#include "ProjectTypes.h"
#include "GpsDiagnostics.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"

class GpsTracker {
public:
    esp_err_t initialize();
    esp_err_t readFix(GpsFix *fix);
    // Call from the same task as readFix(); copy the result to other tasks.
    GpsDiagnostics diagnostics() const;

private:
    void consumeByte(uint8_t byte);
    void parseLine();

    char line_[128] = {};
    size_t lineLength_ = 0;
    GpsFix latestFix_ = {};
    TickType_t lastFixAt_ = 0;
    bool hasFix_ = false;
    bool initialized_ = false;
    bool uartError_ = false;
    bool receivedAny_ = false;
    bool receivedNmea_ = false;
    bool receivedNavigation_ = false;
    bool receivedGga_ = false;
    TickType_t lastRxAt_ = 0;
    TickType_t lastNmeaAt_ = 0;
    TickType_t lastNavigationAt_ = 0;
    TickType_t lastGgaAt_ = 0;
    TickType_t lastSpeedAt_ = 0;
    GpsDiagnostics diagnostics_ = {};
};
