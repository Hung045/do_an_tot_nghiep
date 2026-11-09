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
    bool crashDetected = false; // Latched event; serialized in existing EEPROM byte 29.
};

enum class VehicleEvent : uint8_t {
    CrashDetected,
    PowerWarning,
    SensorFault
};

// Live commands/status only; not part of the EEPROM record layout.
struct LightingTelemetry {
    bool outputValid = false;
    float ambientLux = -1.0F;
    uint32_t dutyPercent = 0;
    bool faultBlink = false;
    bool servoReady = false;
    bool servoTracking = false;
    bool rollValid = false;
    float rollDegrees = 0.0F;
    float servoDegrees = 90.0F;
    float servoTargetDegrees = 90.0F;
};
