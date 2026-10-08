#pragma once

#include "ProjectTypes.h"
#include "esp_err.h"

class TelemetryLink {
public:
    esp_err_t initialize(const char *brokerUri);
    esp_err_t publishTelemetry(const TelemetryRecord &record);
    esp_err_t publishEvent(VehicleEvent event, const GpsFix &fix);
};
