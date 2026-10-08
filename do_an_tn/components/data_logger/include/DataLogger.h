#pragma once

#include <cstddef>
#include <cstdint>

#include "esp_err.h"
#include "ProjectTypes.h"

class DataLogger {
public:
    esp_err_t initialize();
    esp_err_t append(const TelemetryRecord &record);
    esp_err_t readLatest(TelemetryRecord *record);

private:
    esp_err_t readBytes(uint16_t address, uint8_t *data, size_t length);
    esp_err_t writeBytes(uint16_t address, const uint8_t *data, size_t length);

    uint32_t nextIndex_ = 0;
    uint32_t recordCount_ = 0;
    uint32_t sequence_ = 0;
    bool initialized_ = false;
};
