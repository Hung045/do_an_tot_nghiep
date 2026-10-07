#include "PowerMonitor.h"

esp_err_t PowerMonitor::initialize()
{
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t PowerMonitor::read(PowerReading *reading)
{
    if (reading == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    return ESP_ERR_NOT_SUPPORTED;
}
