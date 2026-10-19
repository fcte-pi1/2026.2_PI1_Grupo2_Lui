#pragma once

#include <cstdint>

#include "navigation/PositionTracker.h"
#include "navigation/SessionBuffer.h"

namespace mm {

/**
 * Liga o PositionTracker (HU-02) ao SessionBuffer (HU-11).
 *
 * Chamar onTracking(tracker.currentState()) a cada ciclo, logo após
 * tracker.update(). Só grava um ponto quando a CÉLULA muda (rotação no mesmo
 * lugar não gera ponto), então a rota fica com uma entrada por célula.
 *
 * PREMISSA: TrackingState::timestampMs e os now_ms passados a begin()/end()
 * estão na MESMA base de tempo (millis()).
 */
class SessionRecorder {
public:
    explicit SessionRecorder(SessionBuffer &buffer) : buffer_(buffer) {}

    /** Abre a sessão e grava a célula de partida (t = 0). */
    bool begin(uint32_t session_id, const tracking::TrackingState &initial, uint32_t now_ms);

    /** true se um ponto novo foi gravado. */
    bool onTracking(const tracking::TrackingState &state);

    /** Fecha a sessão (chegada, parada manual ou erro). */
    bool end(uint32_t now_ms, EndReason reason);

    bool running() const { return buffer_.state() == SessionState::Running; }

private:
    SessionBuffer &buffer_;
    tracking::GridPosition last_{};
};

}  // namespace mm
