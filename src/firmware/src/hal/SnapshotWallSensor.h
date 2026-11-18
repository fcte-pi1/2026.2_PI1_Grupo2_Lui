#pragma once
#include <cmath>
#include "IWallSensor.h"
#include "sonar_snapshot.hpp"
#include "config/sonar_sampling.hpp"

// One coherent hardware frame for Front/Left/Right; no duplicate sonar triggers.
// This adapter treats a hardware timeout as UNKNOWN, not an observed opening.
class SnapshotWallSensor final : public perception::IWallSensor {
public:
    using ReadSnapshot = SonarSnapshot (*)();
    using Clock = uint32_t (*)();
    SnapshotWallSensor(ReadSnapshot read, Clock clock) : read_(read), clock_(clock) {}
    perception::RawEcho read(perception::Direction direction) override {
        if (!cached_ || direction == perception::Direction::Front) {
            frame_ = read_();
            fresh_ = frame_.ready && static_cast<uint32_t>(clock_() - frame_.sampledAtMs)
                                      <= SonarSampling::MAX_SAMPLE_AGE_MS;
            cached_ = true;
        }
        UltrasonicReading estimate=frame_.frontEstimate(clock_());
        if(direction==perception::Direction::Left)estimate=frame_.leftEstimate(clock_());
        if(direction==perception::Direction::Right)estimate=frame_.rightEstimate(clock_());
        const UltrasonicReading* value=&estimate;
        perception::RawEcho echo{};
        echo.timestampMs = frame_.sampledAtMs;
        echo.hasEcho = frame_.hasFilters ? value->valid : value->pulseUs != 0;
        echo.preFiltered=frame_.hasFilters;
        echo.valid = fresh_ && value->valid && echo.hasEcho &&
                     !value->tooClose && std::isfinite(value->distanceCm);
        if (std::isfinite(value->distanceCm)) {
            const float mm = std::fmin(65535.0f, std::fmax(0.0f, value->distanceCm * 10.0f));
            echo.rawDistanceMm = static_cast<uint16_t>(std::lround(mm));
        }
        if (direction == perception::Direction::Right) cached_ = false;
        return echo;
    }
private:
    ReadSnapshot read_;
    Clock clock_;
    SonarSnapshot frame_{};
    bool cached_ = false, fresh_ = false;
};
