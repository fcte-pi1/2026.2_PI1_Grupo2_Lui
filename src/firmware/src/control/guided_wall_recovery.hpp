#pragma once

#include <cmath>

class GuidedWallRecovery {
public:
    enum class Phase { None, Braking, BeforeAdjustment, Adjusting, Checking, Resuming };

    void request(bool front, bool left, bool right, bool slowApproach) {
        phase = Phase::Braking;
        frontBlocked = front;
        leftBlocked = left;
        rightBlocked = right;
        approach = slowApproach;
        failedAdjustmentMm = 0.0f;
        centreChoiceMade_ = false;
    }

    // Sem sensor traseiro, recua apenas sobre a parte já percorrida deste trecho.
    float retreatDistance(float travelledMm, float targetMm, float maxRetreatMm = 30.0f) {
        constexpr float marginMm = 5.0f;
        const float minRetreatMm = maxRetreatMm < 30.0f ? 5.0f : 8.0f;
        if (phase != Phase::Braking || !std::isfinite(travelledMm) || !std::isfinite(targetMm))
            return 0.0f;
        travelledBeforeResumeMm_ += travelledMm;
        remainingMm = targetMm - travelledMm;
        // Se o travamento aconteceu na partida, ainda pode haver espaço à frente.
        phase = Phase::Checking;
        const float distance = std::fmin(maxRetreatMm, travelledBeforeResumeMm_ - marginMm);
        if (distance < minRetreatMm || remainingMm + distance <= marginMm) {
            failedAdjustmentMm = -1.0f;
            return 0.0f;
        }
        planAdjustment(-distance);
        return distance;
    }

    void planAdjustment(float signedMm) {
        plannedAdjustmentMm = signedMm;
        phase = Phase::BeforeAdjustment;
    }

    void startAdjustment(float frontCm) {
        frontBeforeRetreatCm_ = frontCm;
        phase = Phase::Adjusting;
    }

    bool clearingFrontBlindZone() const { return frontBlocked && !std::isfinite(frontBeforeRetreatCm_); }

    void observeFront(float frontCm) {
        if (clearingFrontBlindZone() && std::isfinite(frontCm)) frontBeforeRetreatCm_ = frontCm;
    }

    bool frontAllowsRetreat(float frontCm) const {
        // Uma pequena variação do eco é normal; uma queda persistente
        // indica que o robô deixou de se afastar da referência frontal.
        constexpr float toleranceCm = 1.5f;
        return std::isfinite(frontCm) && std::isfinite(frontBeforeRetreatCm_) &&
            frontCm >= frontBeforeRetreatCm_ - toleranceCm;
    }

    void finishAdjustment(float travelledMm, bool succeeded) {
        travelledBeforeResumeMm_ += travelledMm;
        remainingMm -= travelledMm;
        failedAdjustmentMm = succeeded ? 0.0f : plannedAdjustmentMm;
        if (!succeeded) {
            centreChoiceMade_ = true;
            centreBehind_ = false;
        }
        phase = Phase::Checking;
    }

    float availableRetreatMm() const { return std::fmax(0.0f, travelledBeforeResumeMm_ - 5.0f); }
    float originalTargetMm() const { return travelledBeforeResumeMm_ + remainingMm; }

    // O começo de um trecho célula a célula é o centro já percorrido.
    // O centro à frente só é candidato se o eco confirmar espaço até ele.
    float centreRetreatMm(bool startsAtCellCentre, float frontSpaceMm) {
        if (!centreChoiceMade_) {
            centreBehind_ = startsAtCellCentre && availableRetreatMm() >= 4.0f &&
                (travelledBeforeResumeMm_ < remainingMm || frontSpaceMm < remainingMm);
            centreChoiceMade_ = true;
        }
        return centreBehind_ ? std::fmin(30.0f, availableRetreatMm()) : 0.0f;
    }

    float steeringOffset(float leftCm, float rightCm) const {
        if (leftBlocked != rightBlocked) return leftBlocked ? -6.0f : 6.0f;
        if (frontBlocked && std::isfinite(leftCm) && std::isfinite(rightCm) &&
            std::fabs(leftCm - rightCm) > 0.5f)
            return leftCm > rightCm ? 6.0f : -6.0f;
        return 0.0f;
    }

    Phase phase = Phase::None;
    bool frontBlocked = false;
    bool leftBlocked = false;
    bool rightBlocked = false;
    bool approach = false;
    float remainingMm = 0.0f;
    float plannedAdjustmentMm = 0.0f;
    float failedAdjustmentMm = 0.0f;

private:
    bool centreChoiceMade_ = false;
    bool centreBehind_ = false;
    float travelledBeforeResumeMm_ = 0.0f;
    float frontBeforeRetreatCm_ = NAN;
};
