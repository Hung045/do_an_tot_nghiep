#include "DataLogger.h"

esp_err_t DataLogger::initialize()
{
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t DataLogger::append(const TelemetryRecord &record)
{
    (void)record;
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t DataLogger::readLatest(TelemetryRecord *record)
{
    if (record == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    return ESP_ERR_NOT_SUPPORTED;
}
