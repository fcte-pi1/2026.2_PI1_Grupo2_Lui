#pragma once
#include <cmath>

// For the USB-powered, motors-disabled diagnostic only. IMU alone cannot
// distinguish an extremely slow constant rotation from gyro bias.
class DiagnosticYaw {
public:
    void reset() { yaw_ = bias_ = quietSeconds_ = 0; previous_ = false; stationary_ = false; }
    float update(float rate, float ax, float ay, float az, float dt) {
        if (!std::isfinite(rate) || !std::isfinite(ax) || !std::isfinite(ay) ||
            !std::isfinite(az) || !std::isfinite(dt) || dt <= 0 || dt > 0.1f) {
            quietSeconds_ = 0; previous_ = false; stationary_ = false;
            return yaw_;
        }
        const float acceleration = std::sqrt(ax*ax + ay*ay + az*az);
        const float delta = previous_ ? std::sqrt((ax-px_)*(ax-px_) +
            (ay-py_)*(ay-py_) + (az-pz_)*(az-pz_)) : 0;
        const float corrected = rate-bias_;
        const bool quiet = std::fabs(corrected) < 0.25f &&
            std::fabs(acceleration-1.0f) < 0.08f && delta < 0.02f;
        quietSeconds_ = quiet ? quietSeconds_ + dt : 0;
        stationary_ = quietSeconds_ >= 1.0f;
        if (stationary_) {
            // Slowly track residual temperature/bias changes, only at rest.
            bias_ += (1.0f-std::exp(-dt/5.0f)) * corrected;
        } else {
            yaw_ += corrected*dt;
            while (yaw_ >= 180) yaw_ -= 360;
            while (yaw_ < -180) yaw_ += 360;
        }
        px_=ax; py_=ay; pz_=az; previous_=true;
        return yaw_;
    }
    bool stationary() const { return stationary_; }
private:
    float yaw_=0, bias_=0, quietSeconds_=0, px_=0, py_=0, pz_=0;
    bool previous_=false, stationary_=false;
};
