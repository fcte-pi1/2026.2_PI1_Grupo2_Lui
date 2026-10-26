#pragma once

#include <cstdint>

#include "navigation/SessionBuffer.h"

namespace mm {

/**
 * Registro de percurso da HU-11. Recebe uma pose estrutural compatível com o
 * TrackingState da HU-02, mas não inclui PositionTracker.h diretamente: isso
 * mantém a gravação desacoplada do algoritmo de estimativa da pose.
 *
 * Qualquer estado com position.row, position.col, orientation e timestampMs
 * pode ser passado às sobrecargas template, incluindo tracking::TrackingState.
 */
struct SessionPose {
    int16_t row = 0;
    int16_t col = 0;
    uint8_t heading = 0; // 0=N, 1=E, 2=S, 3=W
    uint32_t timestampMs = 0;
};

class SessionRecorder {
public:
    explicit SessionRecorder(SessionBuffer &buffer) : buffer_(buffer) {}

    bool begin(uint32_t session_id, const SessionPose &initial, uint32_t now_ms,
               const char *tipo_labirinto = "desconhecido", uint16_t tentativa = 1);
    bool onTracking(const SessionPose &state);

    template <typename TrackingState>
    bool begin(uint32_t session_id, const TrackingState &initial, uint32_t now_ms,
               const char *tipo_labirinto = "desconhecido", uint16_t tentativa = 1) {
        return begin(session_id, adapt(initial), now_ms, tipo_labirinto, tentativa);
    }

    template <typename TrackingState>
    bool onTracking(const TrackingState &state) {
        return onTracking(adapt(state));
    }

    /** Fecha a sessão (chegada, parada manual ou erro). */
    bool end(uint32_t now_ms, EndReason reason);

    bool running() const { return buffer_.state() == SessionState::Running; }

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
    int16_t last_row_ = 0;
    int16_t last_col_ = 0;
};

}  // namespace mm
