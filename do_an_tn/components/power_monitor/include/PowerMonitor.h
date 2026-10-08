#pragma once

#include "esp_err.h"

struct PowerReading {
    float busVoltageVolts;
    float currentMilliamps;
    float powerMilliwatts;
    float stateOfChargePercent;
};

class PowerMonitor {
public:
    esp_err_t initialize();
    esp_err_t read(PowerReading *reading);

private:
    float currentLsbAmps_ = 0.0F;
};
