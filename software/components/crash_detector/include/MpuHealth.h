#pragma once
#include <cstdint>

// Pure timing policy; the caller must stop motor outputs while recovering.
class MpuHealth {
public:
    enum class State { Healthy, Recovering, Fault };
    State state() const { return state_; }
    State update(bool valid, uint32_t nowMs)
    {
        if (state_ == State::Fault) return state_;
        if (state_ == State::Healthy) {
            if (valid) return state_;
            state_ = State::Recovering;
            failedAt_ = nowMs;
        }
        if (static_cast<uint32_t>(nowMs - failedAt_) >= 200U) {
            state_ = State::Fault;
            return state_;
        }
        if (!valid) {
            goodSamples_ = 0;
        } else {
            if (goodSamples_ == 0) goodSince_ = nowMs;
            if (goodSamples_ < 3) ++goodSamples_;
            if (goodSamples_ >= 3 && static_cast<uint32_t>(nowMs - goodSince_) >= 40U)
                state_ = State::Healthy;
        }
        return state_;
    }
private:
    State state_ = State::Healthy;
    uint32_t failedAt_ = 0;
    uint32_t goodSince_ = 0;
    unsigned goodSamples_ = 0;
};
