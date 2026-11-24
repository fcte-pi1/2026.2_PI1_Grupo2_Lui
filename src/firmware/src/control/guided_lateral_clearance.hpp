#pragma once

#include <algorithm>
#include <cmath>
#include "corridor_feedback_filter.hpp"
#include "../hal/sonar_snapshot.hpp"

// Protege os avanços guiados mesmo quando só uma parede devolve eco.
class GuidedLateralClearance {
public:
    static constexpr float MIN_CLEARANCE_CM = 1.9f;
    static constexpr float STEERING_CLEARANCE_CM = 4.0f;

    struct Feedback {
        bool slow = false;
        float yawOffsetDeg = 0.0f;
        bool leftBlocked = false;
        bool rightBlocked = false;
        bool leftUncertain = false;
        bool rightUncertain = false;
        float rightProjectedCm = NAN;
    };

    Feedback update(const SonarSnapshot& snapshot) {
        if (hasFrame_ && snapshot.sampledAtMs == sampledAtMs_) return feedback_;

        const uint32_t elapsedMs = snapshot.sampledAtMs - sampledAtMs_;
        const bool consecutive = hasFrame_ && elapsedMs <= SonarSampling::MAX_SAMPLE_AGE_MS;
        sampledAtMs_ = snapshot.sampledAtMs;
        hasFrame_ = true;
        const bool leftWasNear = consecutive && left_.projectedCm <= STEERING_CLEARANCE_CM;
        const bool rightWasNear = consecutive && right_.projectedCm <= STEERING_CLEARANCE_CM;
        assess(snapshot.left, left_, consecutive, elapsedMs, feedback_.leftBlocked, feedback_.leftUncertain);
        assess(snapshot.right, right_, consecutive, elapsedMs, feedback_.rightBlocked, feedback_.rightUncertain);
        feedback_.rightProjectedCm = right_.projectedCm;
        const float left = pressure(left_.projectedCm);
        const float right = pressure(right_.projectedCm);
        feedback_.slow = left > 0.0f || right > 0.0f ||
            (feedback_.leftUncertain && leftWasNear) || (feedback_.rightUncertain && rightWasNear);
        const auto pair = corridor_.update(left_.measuredCm * 10.0f, right_.measuredCm * 10.0f,
                                           true, sampledAtMs_, sampledAtMs_);
        float errorCm = right - left;
        if (pair.valid) {
            centeredClearanceCm_ = (pair.leftMm + pair.rightMm) * 0.05f;
            const float centerErrorCm = (pair.leftMm - pair.rightMm) * 0.05f;
            const float closingErrorCm = errorCm - (pressure(right_.measuredCm) - pressure(left_.measuredCm));
            errorCm = centerError(centerErrorCm) + closingErrorCm;
        } else if (std::isfinite(centeredClearanceCm_)) {
            // Na abertura, a parede que continua serve de referência para o centro do corredor.
            constexpr float corridorWallMaxCm = 8.0f;
            if (left_.measuredCm > corridorWallMaxCm && right_.measuredCm <= corridorWallMaxCm)
                errorCm = centerError(centeredClearanceCm_ - right_.projectedCm);
            else if (right_.measuredCm > corridorWallMaxCm && left_.measuredCm <= corridorWallMaxCm)
                errorCm = centerError(left_.projectedCm - centeredClearanceCm_);
        }
        constexpr float brakingReserveCm = MIN_CLEARANCE_CM + 0.5f;
        const float leftUrgency = std::max(0.0f, brakingReserveCm - left_.projectedCm);
        const float rightUrgency = std::max(0.0f, brakingReserveCm - right_.projectedCm);
        // Yaw positivo gira à esquerda, afastando-se da parede direita.
        feedback_.yawOffsetDeg = std::clamp(6.0f * (errorCm + rightUrgency - leftUrgency), -6.0f, 6.0f);
        return feedback_;
    }

    void reset() { *this = GuidedLateralClearance{}; }

    bool oppositeWallOffset(bool leftBlocked, bool rightBlocked, float leftCm, float rightCm,
                            float& offsetDeg) const {
        if (leftBlocked == rightBlocked) return false;
        const float oppositeCm = leftBlocked ? rightCm : leftCm;
        if (!std::isfinite(oppositeCm) || oppositeCm <= MIN_CLEARANCE_CM || oppositeCm > 8.0f) return false;
        const float targetCm = std::isfinite(centeredClearanceCm_) ? centeredClearanceCm_ : 3.0f;
        const float errorCm = leftBlocked ? targetCm - oppositeCm : oppositeCm - targetCm;
        offsetDeg = std::clamp(6.0f * centerError(errorCm), -6.0f, 6.0f);
        return true;
    }

private:
    struct Side {
        float measuredCm = NAN;
        float projectedCm = NAN;
    };

    static float pressure(float cm) {
        return std::max(0.0f, STEERING_CLEARANCE_CM - cm);
    }

    static float centerError(float cm) {
        constexpr float deadbandCm = 0.3f;
        return cm - std::clamp(cm, -deadbandCm, deadbandCm);
    }

    static void assess(const UltrasonicReading& reading, Side& previous,
                        bool consecutive, uint32_t elapsedMs, bool& blocked, bool& uncertain) {
        const bool valid = reading.valid && !reading.tooClose && reading.pulseUs && std::isfinite(reading.distanceCm);
        blocked = reading.tooClose || (valid && reading.distanceCm <= MIN_CLEARANCE_CM);
        uncertain = !valid && !blocked;
        if (!valid) {
            previous = {};
            return;
        }

        float projectedCm = reading.distanceCm;
        if (consecutive && std::isfinite(previous.measuredCm)) {
            // Projeta o fechamento pela janela de idade admitida para um quadro.
            const float closingCmS = (previous.measuredCm - reading.distanceCm) * 1000.0f / elapsedMs;
            constexpr float horizonS = SonarSampling::MAX_SAMPLE_AGE_MS / 1000.0f;
            projectedCm -= std::clamp(closingCmS, 0.0f, 5.0f) * horizonS;
        }
        previous = {reading.distanceCm, projectedCm};
    }

    Side left_, right_;
    CorridorFeedbackFilter3 corridor_;
    float centeredClearanceCm_ = NAN;
    Feedback feedback_;
    uint32_t sampledAtMs_ = 0;
    bool hasFrame_ = false;
};
