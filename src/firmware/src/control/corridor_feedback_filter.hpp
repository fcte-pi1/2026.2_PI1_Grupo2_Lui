#pragma once
#include <array>
#include <cmath>
#include <cstdint>
#include "config/sonar_sampling.hpp"

// Only used for lateral steering. Front emergency checks bypass this window.
// A window consists of distinct hardware frames, never repeated control ticks.
class CorridorFeedbackFilter3 {
public:
    struct Result {
        bool changed = false;
        bool valid = false;
        float leftMm = 0;
        float rightMm = 0;
    };

    void reset() { *this = CorridorFeedbackFilter3{}; }

    Result update(float left, float right, bool valid, uint32_t sampledAt,
                  uint32_t now) {
        if (static_cast<uint32_t>(now - sampledAt) > SonarSampling::MAX_SAMPLE_AGE_MS) {
            clearWindow();
            return {true, false, 0, 0};
        }
        if (hasTimestamp_ && sampledAt == lastTimestamp_) return {};
        if (hasTimestamp_ && static_cast<uint32_t>(sampledAt - lastTimestamp_) >
                             SonarSampling::MAX_SAMPLE_AGE_MS) clearWindow();
        lastTimestamp_ = sampledAt;
        hasTimestamp_ = true;
        // Check the CURRENT echoes before filtering. An opening or invalid
        // echo must disable steering immediately, not after a median catches up.
        if (!valid || !std::isfinite(left) || !std::isfinite(right) ||
            left < 20 || right < 20 || left > 80 || right > 80 ||
            left + right < 40 || left + right > 120) {
            clearWindow();
            return {true, false, 0, 0};
        }
        left_[cursor_] = left;
        right_[cursor_] = right;
        cursor_ = (cursor_ + 1) % 3;
        if (count_ < 3) ++count_;
        if (count_ < 3) return {true, false, 0, 0};
        return {true, true, median(left_), median(right_)};
    }

private:
    void clearWindow() { cursor_ = count_ = 0; }
    static float median(std::array<float, 3> v) {
        if (v[0] > v[1]) { float t = v[0]; v[0] = v[1]; v[1] = t; }
        if (v[1] > v[2]) { float t = v[1]; v[1] = v[2]; v[2] = t; }
        return v[0] > v[1] ? v[0] : v[1];
    }
    std::array<float, 3> left_{}, right_{};
    uint8_t cursor_ = 0, count_ = 0;
    bool hasTimestamp_ = false;
    uint32_t lastTimestamp_ = 0;
};
