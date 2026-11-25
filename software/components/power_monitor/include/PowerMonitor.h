#pragma once

#include <cstdint>

#include "esp_err.h"

struct PowerReading {
    float busVoltageVolts;
    float currentMilliamps;
    float powerMilliwatts;
    float stateOfChargePercent;
    float stateOfHealthPercent;
};

class PowerMonitor {
public:
    esp_err_t initialize();
    esp_err_t read(PowerReading *reading);

private:
    void updateSohCycle(float voltage, float dischargeMilliamps);

    bool initialized_ = false;
    bool hasFilteredCurrent_ = false;
    bool sohCycleActive_ = false;
    bool sohFullReached_ = false;
    float currentLsbAmps_ = 0.0F;
    float filteredCurrentMilliamps_ = 0.0F;
    float learnedCapacityMah_ = -1.0F;
    float stateOfHealthPercent_ = -1.0F;
    float cycleDischargeMah_ = 0.0F;
    uint32_t lastSohUpdateTick_ = 0;
    uint16_t calibration_ = 0;
};
