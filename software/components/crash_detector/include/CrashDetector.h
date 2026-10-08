#pragma once

#include "esp_err.h"
#include "freertos/FreeRTOS.h"

class CrashDetector {
public:
    esp_err_t initialize();
    esp_err_t update(float *tiltDegrees, bool *crashDetected);

private:
    esp_err_t readTiltDegrees(float *tiltDegrees);

    bool tiltTiming_ = false;
    TickType_t tiltDetectedAt_ = 0;
};
