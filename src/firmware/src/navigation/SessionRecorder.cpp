#include "navigation/SessionRecorder.h"

namespace mm {

bool SessionRecorder::begin(uint32_t session_id, const SessionPose &initial,
                            uint32_t now_ms, const char *tipo_labirinto, uint16_t tentativa)
{
    if (!buffer_.start(session_id, now_ms, tipo_labirinto, tentativa)) {
        return false;
    }
    last_row_ = initial.row;
    last_col_ = initial.col;
    return buffer_.record(now_ms, initial.col, initial.row, initial.heading);
}

bool SessionRecorder::onTracking(const SessionPose &state)
{
    if (!running()) {
        return false;
    }
    if (state.row == last_row_ && state.col == last_col_) {
        return false; // rotação na mesma célula não cria ponto redundante
    }
    last_row_ = state.row;
    last_col_ = state.col;

    // Timestamp anterior ao início é fixado em t=0; a subtração respeita wrap de millis().
    uint32_t ts = state.timestampMs;
    if (static_cast<int32_t>(ts - buffer_.startMs()) < 0) {
        ts = buffer_.startMs();
    }
    return buffer_.record(ts, state.col, state.row, state.heading);
}

bool SessionRecorder::end(uint32_t now_ms, EndReason reason)
{
    return buffer_.stop(now_ms, reason);
}

}  // namespace mm
