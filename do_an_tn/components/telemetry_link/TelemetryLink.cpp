#include "TelemetryLink.h"

esp_err_t TelemetryLink::initialize(const char *brokerUri)
{
    if (brokerUri == nullptr || brokerUri[0] == '\0') {
        return ESP_ERR_INVALID_ARG;
    }
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t TelemetryLink::publishTelemetry(const TelemetryRecord &record)
{
    (void)record;
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t TelemetryLink::publishEvent(VehicleEvent event, const GpsFix &fix)
{
    (void)event;
    (void)fix;
    return ESP_ERR_NOT_SUPPORTED;
}
