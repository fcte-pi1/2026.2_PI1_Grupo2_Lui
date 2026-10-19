#pragma once

#include <array>
#include <cstdint>

#include "navigation/PositionTracker.h"
#include "navigation/WallPerception.h"

// Integra as HU-01 (percepção relativa de paredes) e HU-02 (pose absoluta na
// grade), sem duplicar a leitura dos sensores, os filtros ou a odometria.
namespace state_estimation {

enum class WallKnowledge : uint8_t {
    Unknown,  // sensor inválido ou direção sem sensor (traseira)
    Open,     // leitura válida indica ausência de parede
    Present   // leitura válida indica parede
};

struct AbsoluteWallObservation {
    WallKnowledge knowledge = WallKnowledge::Unknown;
    uint16_t distanceMm = 0;
    bool distanceValid = false;
};

struct RobotStateSnapshot {
    tracking::TrackingState pose{};
    // Índices usam a ordem de tracking::Orientation: North, East, South, West.
    std::array<AbsoluteWallObservation, 4> walls{};
    uint32_t sensorTimestampMs = 0;
    bool initialized = false;

    const AbsoluteWallObservation& wallAt(tracking::Orientation direction) const;
};

// Chamar uma vez por ciclo de controle. O snapshot combina pose e paredes
// absolutas consistentes com a orientação mais recente do robô.
class RobotStateEstimator {
public:
    RobotStateEstimator(tracking::PositionTracker& positionTracker,
                        perception::WallPerception& wallPerception);

    void update();
    const RobotStateSnapshot& currentState() const { return state_; }

private:
    tracking::PositionTracker& positionTracker_;
    perception::WallPerception& wallPerception_;
    RobotStateSnapshot state_{};
};

}  // namespace state_estimation
