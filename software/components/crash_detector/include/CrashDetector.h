#pragma once

#include "esp_err.h"
#include "freertos/FreeRTOS.h"

class CrashDetector {
public:
    esp_err_t initialize();
    esp_err_t update(float *tiltDegrees, bool *crashDetected);
    bool isAvailable() const { return available_; }
    // Cached from the same successful read as update(); call in its owning task.
    float rollDegrees() const { return rollDegrees_; }

private:
    esp_err_t readTiltDegrees(float *tiltDegrees);

    bool tiltTiming_ = false;
    bool available_ = false;
    TickType_t tiltDetectedAt_ = 0;
    float rollDegrees_ = 0.0F;
};
