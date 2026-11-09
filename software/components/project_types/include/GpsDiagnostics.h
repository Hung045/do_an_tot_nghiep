#pragma once

#include <cstdint>

enum class GpsStatus : uint8_t {
    InitError, NoData, InvalidNmea, NoNavigation, NoFix, FixOk, Stale, UartError
};

// Live diagnostics only: kept separate from the EEPROM record format.
struct GpsDiagnostics {
    GpsStatus status = GpsStatus::InitError;
    uint32_t receivedBytes = 0;
    uint32_t validSentences = 0;
    uint32_t rejectedSentences = 0;
    int32_t rxAgeMs = -1;
    int32_t nmeaAgeMs = -1;
};

inline const char *gpsStatusName(GpsStatus status)
{
    switch (status) {
    case GpsStatus::InitError: return "INIT_ERROR";
    case GpsStatus::NoData: return "NO_DATA";
    case GpsStatus::InvalidNmea: return "INVALID_NMEA";
    case GpsStatus::NoNavigation: return "NO_NAV";
    case GpsStatus::NoFix: return "NO_FIX";
    case GpsStatus::FixOk: return "FIX_OK";
    case GpsStatus::Stale: return "STALE";
    case GpsStatus::UartError: return "UART_ERROR";
    }
    return "INIT_ERROR";
}
