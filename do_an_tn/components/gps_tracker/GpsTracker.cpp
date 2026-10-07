#include "GpsTracker.h"

esp_err_t GpsTracker::initialize()
{
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t GpsTracker::readFix(GpsFix *fix)
{
    if (fix == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    return ESP_ERR_NOT_SUPPORTED;
}
