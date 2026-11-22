#pragma once
#include <cstdint>
#include <cmath>
#include "../config/maze_geometry.hpp"
#include "../control/front_approach_policy.hpp"
namespace nav {
// Keep 8cm of measured frontal space before changing to the 2cm/s approach.
// A cell-length step is a maximum, never permission to drive through a wall.
inline float guidedPreCornerMoveMm(float frontCm, float cellMm = MazeSettings::CELL_SIZE_MM) {
    if (!std::isfinite(frontCm)) return 0;
    return std::fmin(cellMm, std::fmax(0.0f, (frontCm - 8.0f) * 10.0f));
}

enum class GuidedCornerAction : uint8_t { Advance, ApproachSlowly, AtTarget, TooClose, NoTravelLeft };
struct GuidedCornerPlan {
    GuidedCornerAction action;
    float distanceMm = 0.0f;
};

inline bool guidedFrontAtCorner(float frontCm) {
    return std::fabs(frontCm - MazeSettings::GUIDED_FRONT_TARGET_CM) <= MazeSettings::GUIDED_FRONT_TOLERANCE_CM;
}

// Recebe uma estimativa frontal válida. Cada avanço respeita a capacidade do
// controlador (uma célula) e o orçamento restante até a curva.
inline GuidedCornerPlan planGuidedCorner(float frontCm, float cellMm, float remainingMm) {
    if (guidedFrontAtCorner(frontCm)) return {GuidedCornerAction::AtTarget};
    if (frontCm < MazeSettings::GUIDED_FRONT_TARGET_CM) return {GuidedCornerAction::TooClose};
    const float transitMm = guidedPreCornerMoveMm(frontCm, cellMm);
    const bool transit = frontCm > FrontApproach::START_CM;
    const float desiredMm = transit ? transitMm :
        (frontCm - MazeSettings::GUIDED_FRONT_TARGET_CM) * 10.0f;
    const float distanceMm = std::fmin(desiredMm, std::fmin(cellMm, remainingMm));
    if (distanceMm <= 1.0f) return {GuidedCornerAction::NoTravelLeft};
    return {transit ? GuidedCornerAction::Advance : GuidedCornerAction::ApproachSlowly, distanceMm};
}

inline bool guidedLeftOpening(float leftCm, float cellMm) {
    return std::isfinite(leftCm) && leftCm * 10.0f > cellMm * MazeSettings::WALL_THRESHOLD_CELL_RATIO;
}

inline bool missionCornerEligible(float frontCm, float exitCm, float cellMm) {
    return frontCm >= MazeSettings::GUIDED_FRONT_TARGET_CM - MazeSettings::GUIDED_FRONT_TOLERANCE_CM &&
        frontCm <= 5.0f && guidedLeftOpening(exitCm, cellMm);
}

// Parâmetros iniciais do ensaio. Pare antes do ponto de contato observado
// (~79°), desloque o eixo em direção à saída e só então complete a curva.
constexpr float GUIDED_CORNER_ENTRY_DEG = 65.0f;
constexpr float GUIDED_CORNER_EXIT_DEG = 90.0f - GUIDED_CORNER_ENTRY_DEG;
constexpr float GUIDED_CORNER_EXIT_RAD = GUIDED_CORNER_EXIT_DEG * 0.01745329252f;
constexpr float GUIDED_CORNER_RIGHT_TARGET_CM = 2.0f;
constexpr float GUIDED_CORNER_RIGHT_MIN_CM = 1.7f;
constexpr float GUIDED_CORNER_RIGHT_START_CM = 4.5f;
constexpr float GUIDED_CORNER_MIN_SHIFT_MM = 27.0f;
constexpr float GUIDED_CORNER_SHIFT_TOLERANCE_MM = 2.0f;

inline float guidedCornerRightGapCm(float rightEchoCm) {
    return rightEchoCm * std::cos(GUIDED_CORNER_EXIT_RAD);
}

inline bool guidedCornerBrakeAtRightTarget(float rightEchoCm, float projectedEchoCm) {
    if (!std::isfinite(rightEchoCm)) return false;
    // Use só parte da projeção para antecipar a frenagem sem parar
    // desnecessariamente longe da parede.
    const float brakeEchoCm = rightEchoCm + 0.4f * std::fmin(0.0f, projectedEchoCm - rightEchoCm);
    return guidedCornerRightGapCm(brakeEchoCm) <= GUIDED_CORNER_RIGHT_TARGET_CM;
}

inline float guidedCornerShiftMm(float frontCm, float rightCm) {
    if (!std::isfinite(frontCm) || !std::isfinite(rightCm) || rightCm < GUIDED_CORNER_RIGHT_START_CM)
        return 0.0f;
    constexpr float maxShiftMm = 50.0f;
    constexpr float reserveFrontCm = 5.0f;
    // A leitura direita inicial autoriza a aproximação. O eco durante a
    // diagonal decide a frenagem antes que os encoders esgotem o percurso.
    const float distance = std::fmin(maxShiftMm, (frontCm - reserveFrontCm) * 10.0f);
    return distance >= GUIDED_CORNER_MIN_SHIFT_MM ? distance : 0.0f;
}

inline bool guidedCornerShiftReached(float travelledMm) {
    return travelledMm >= GUIDED_CORNER_MIN_SHIFT_MM - GUIDED_CORNER_SHIFT_TOLERANCE_MM;
}

// Reposicionamento depois de retornar ao rumo da diagonal. A ré preserva
// o avanço mínimo para o giro e o avanço respeita a reserva perpendicular.
inline float guidedCornerRepositionMm(bool rightBlocked, float frontCm, float rightCm, float travelledMm) {
    if (!std::isfinite(frontCm) || !std::isfinite(travelledMm) || frontCm <= 5.0f) return 0.0f;
    if (rightBlocked) {
        const float room = travelledMm - (GUIDED_CORNER_MIN_SHIFT_MM - GUIDED_CORNER_SHIFT_TOLERANCE_MM) - 0.5f;
        const float retreat = std::fmin(10.0f, room);
        return retreat >= 4.0f ? -retreat : 0.0f;
    }
    if (!std::isfinite(rightCm)) return 0.0f;
    // Margem para um rumo alguns graus mais inclinado que o nominal.
    const float roomMm = (guidedCornerRightGapCm(rightCm) - GUIDED_CORNER_RIGHT_MIN_CM) * 10.0f / 0.71f;
    const float advance = std::fmin(12.0f, std::fmin(65.0f - travelledMm,
        std::fmin(roomMm, (frontCm - 5.0f) * 10.0f)));
    return advance >= 4.0f ? advance : 0.0f;
}

inline float guidedCornerRetreatLimitMm(float travelledMm) {
    constexpr float settlingMarginMm = 0.5f;
    const float availableMm = travelledMm - (GUIDED_CORNER_MIN_SHIFT_MM - GUIDED_CORNER_SHIFT_TOLERANCE_MM)
        - settlingMarginMm;
    return availableMm >= 5.0f ? std::fmin(availableMm, 12.0f) : 30.0f;
}

inline float guidedCornerExitProgressMm(float shiftMm) {
    return shiftMm * std::cos(GUIDED_CORNER_EXIT_RAD);
}

enum class GuidedAction : uint8_t {
    ForwardCell, ApproachFrontWall, BeginLeftTurn, ShiftIntoCorner, FinishLeftTurn, Complete, Failed
};
// Relative movement test; deliberately independent of the maze solver/map.
class GuidedRoute {
public:
    void reset() { step_ = 0; failed_ = false; }
    void resetAtCorner() { reset(); step_ = 2; }
    GuidedAction action() const {
        if (failed_) return GuidedAction::Failed;
        if (step_ >= 7) return GuidedAction::Complete;
        if (step_ == 2) return GuidedAction::ApproachFrontWall;
        if (step_ == 3) return GuidedAction::BeginLeftTurn;
        if (step_ == 4) return GuidedAction::ShiftIntoCorner;
        if (step_ == 5) return GuidedAction::FinishLeftTurn;
        return GuidedAction::ForwardCell;
    }
    void finishMovement(bool success) {
        if (action() == GuidedAction::Complete || failed_) return;
        if (!success) failed_ = true; else ++step_;
    }
    uint8_t step() const { return step_; }
private:
    uint8_t step_ = 0;
    bool failed_ = false;
};
} // namespace nav
