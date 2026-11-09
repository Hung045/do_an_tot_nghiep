#pragma once

#include <cstdint>
#include "esp_err.h"

// Owned by the control task. No feedback sensor: angle is a command, not measured.
class AdaptiveServo {
public:
    esp_err_t initialize();
    esp_err_t update(float rollDegrees, bool imuValid, bool fault, uint32_t elapsedMs);
    bool isReady() const { return ready_; }
    float angleDegrees() const { return angle_; }
    float targetDegrees() const { return target_; }
    bool tracking() const { return tracking_; }

private:
    esp_err_t writeAngle(float degrees);
    bool ready_ = false;
    bool tracking_ = false;
    float angle_ = 90.0F;
    float target_ = 90.0F;
    float filteredRoll_ = 0.0F;
};
