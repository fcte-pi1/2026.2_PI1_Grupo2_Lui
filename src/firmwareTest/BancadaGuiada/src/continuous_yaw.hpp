#pragma once

// Acumula deslocamento angular sem saltar ao cruzar -180/+180 graus.
class ContinuousTurnYaw {
public:
    void reset(float rawYaw = 0.0f) { previousRaw_ = rawYaw; progress_ = 0.0f; }
    float update(float rawYaw, int directionSign) {
        float delta = rawYaw - previousRaw_;
        while (delta > 180.0f) delta -= 360.0f;
        while (delta < -180.0f) delta += 360.0f;
        previousRaw_ = rawYaw;
        progress_ -= delta * directionSign;
        return progress_;
    }
private:
    float previousRaw_ = 0.0f;
    float progress_ = 0.0f;
};
