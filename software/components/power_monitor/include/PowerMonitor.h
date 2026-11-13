#pragma once

#include <cstdint>

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
    bool initialized_ = false;
    bool hasFilteredCurrent_ = false;
    float currentLsbAmps_ = 0.0F;
    float filteredCurrentMilliamps_ = 0.0F;
    uint16_t calibration_ = 0;
};
