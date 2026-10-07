#pragma once

#include "ProjectTypes.h"
#include "esp_err.h"

class GpsTracker {
public:
    esp_err_t initialize();
    esp_err_t readFix(GpsFix *fix);
};
