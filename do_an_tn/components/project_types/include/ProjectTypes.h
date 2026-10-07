#pragma once

#include <cstdint>

struct GpsFix {
    double latitudeDegrees;
    double longitudeDegrees;
    float speedMetersPerSecond;
    uint32_t satellitesInView;
    bool valid;
};

struct TelemetryRecord {
    uint32_t uptimeMilliseconds;
    float tiltDegrees;
    uint32_t throttlePercent;
    float busVoltageVolts;
    float currentMilliamps;
    GpsFix gps;
};

enum class VehicleEvent : uint8_t {
    CrashDetected,
    PowerWarning,
    SensorFault
};
