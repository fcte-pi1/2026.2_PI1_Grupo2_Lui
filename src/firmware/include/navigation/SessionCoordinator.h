#pragma once

#include <cstdint>

#include "communication/Logger.h"
#include "navigation/SessionRecorder.h"

namespace mm {

/**
 * Integra HU-11 e HU-12: mantém um único ID para a gravação do percurso e
 * para todos os eventos. Apenas registra início/fim automaticamente; logs por
 * célula seriam redundantes, pois a própria sequência do percurso já contém
 * essa informação.
 *
 * O ILogTransport deve enfileirar a mensagem sem bloquear a tarefa de navegação.
 */
class SessionCoordinator {
public:
    SessionCoordinator(SessionBuffer &buffer, Logger &logger)
        : buffer_(buffer), recorder_(buffer), logger_(logger) {}

    bool begin(uint32_t sessionId, const SessionPose &initial, uint32_t nowMs,
               const char *tipoLabirinto = "desconhecido", uint16_t tentativa = 1);

    template <typename TrackingState>
    bool begin(uint32_t sessionId, const TrackingState &initial, uint32_t nowMs,
               const char *tipoLabirinto = "desconhecido", uint16_t tentativa = 1) {
        return begin(sessionId, adapt(initial), nowMs, tipoLabirinto, tentativa);
    }

    bool onTracking(const SessionPose &state) { return recorder_.onTracking(state); }

    template <typename TrackingState>
    bool onTracking(const TrackingState &state) {
        return recorder_.onTracking(state);
    }

    bool finish(uint32_t nowMs, EndReason reason);
    const SessionBuffer &session() const { return buffer_; }

private:
    template <typename TrackingState>
    static SessionPose adapt(const TrackingState &state) {
        SessionPose pose;
        pose.row = static_cast<int16_t>(state.position.row);
        pose.col = static_cast<int16_t>(state.position.col);
        pose.heading = static_cast<uint8_t>(state.orientation) & 0x3;
        pose.timestampMs = static_cast<uint32_t>(state.timestampMs);
        return pose;
    }

    SessionBuffer &buffer_;
    SessionRecorder recorder_;
    Logger &logger_;
};

}  // namespace mm
