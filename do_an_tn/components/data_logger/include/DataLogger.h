#pragma once

#include "esp_err.h"
#include "ProjectTypes.h"

class DataLogger {
public:
    esp_err_t initialize();
    esp_err_t append(const TelemetryRecord &record);
    esp_err_t readLatest(TelemetryRecord *record);
};
