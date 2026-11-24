#pragma once
#include <cmath>
#include "IWallSensor.h"
#include "sonar_snapshot.hpp"
#include "../config/sonar_sampling.hpp"

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
        // Encostado na parede o sonar fica abaixo do alcance mínimo: a leitura vira
        // "perto demais" ou some logo depois de uma medida curta. Isso é parede certa.
        const int index = static_cast<int>(direction);
        const UltrasonicReading& raw = direction == perception::Direction::Left ? frame_.left :
            direction == perception::Direction::Right ? frame_.right : frame_.front;
        const uint32_t now = clock_();
        if (echo.valid && echo.rawDistanceMm < NEAR_WALL_MM) { lastNearMs_[index] = now; lastNearMm_[index] = echo.rawDistanceMm; }
        if (!echo.valid && fresh_ && index >= 0 && index < 3) {
            const bool recentNear = lastNearMs_[index] != 0 && now - lastNearMs_[index] <= NEAR_HOLD_MS;
            if (raw.tooClose || recentNear) {
                echo.valid = true;
                echo.hasEcho = true;
                echo.rawDistanceMm = raw.tooClose ? TOUCH_DISTANCE_MM : lastNearMm_[index];
            }
        }
        // Paredes laterais medidas durante a reta, quando os sensores laterais (na frente
        // do robô, ~85 mm à frente do eixo) passaram pelo centro da célula.
        const bool left = direction == perception::Direction::Left;
        if (sideOverride_ && fresh_ && direction != perception::Direction::Front &&
            (left ? overrideLeftValid_ : overrideRightValid_)) {
            echo.valid = true;
            echo.hasEcho = true;
            echo.preFiltered = true;
            echo.rawDistanceMm = left ? overrideLeftMm_ : overrideRightMm_;
        }
        if (direction == perception::Direction::Right) cached_ = false;
        return echo;
    }
    // Lado sem medida válida segue a leitura ao vivo.
    void setSideOverride(bool leftValid, uint16_t leftMm, bool rightValid, uint16_t rightMm) {
        sideOverride_ = leftValid || rightValid;
        overrideLeftValid_ = leftValid; overrideRightValid_ = rightValid;
        overrideLeftMm_ = leftMm; overrideRightMm_ = rightMm;
    }
    void clearSideOverride() { sideOverride_ = false; }
    bool sideOverrideActive() const { return sideOverride_; }
private:
    bool sideOverride_ = false, overrideLeftValid_ = false, overrideRightValid_ = false;
    uint16_t overrideLeftMm_ = 0, overrideRightMm_ = 0;
    static constexpr uint16_t NEAR_WALL_MM = 30;
    static constexpr uint16_t TOUCH_DISTANCE_MM = 20; // mínimo aceito por WallPerception
    static constexpr uint32_t NEAR_HOLD_MS = 1000;
    uint32_t lastNearMs_[3] = {0, 0, 0};
    uint16_t lastNearMm_[3] = {0, 0, 0};
    ReadSnapshot read_;
    Clock clock_;
    SonarSnapshot frame_{};
    bool cached_ = false, fresh_ = false;
};
