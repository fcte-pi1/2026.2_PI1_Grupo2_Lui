#include "navigation/SessionRecorder.h"

namespace mm {

bool SessionRecorder::begin(uint32_t session_id, const tracking::TrackingState &initial,
                            uint32_t now_ms)
{
    if (!buffer_.start(session_id, now_ms)) {
        return false;
    }
    last_ = initial.position;
    return buffer_.record(now_ms, initial.position.col, initial.position.row,
                          static_cast<uint8_t>(initial.orientation));
}

bool SessionRecorder::onTracking(const tracking::TrackingState &state)
{
    if (!running()) {
        return false;
    }
    if (state.position.row == last_.row && state.position.col == last_.col) {
        return false;                          /* mesma célula: nada a gravar */
    }
    last_ = state.position;

    /* Timestamp atrasado em relação ao início não pode gerar t negativo. */
    uint32_t ts = state.timestampMs;
    if (static_cast<int32_t>(ts - buffer_.startMs()) < 0) {
        ts = buffer_.startMs();
    }
    return buffer_.record(ts, state.position.col, state.position.row,
                          static_cast<uint8_t>(state.orientation));
}

bool SessionRecorder::end(uint32_t now_ms, EndReason reason)
{
    return buffer_.stop(now_ms, reason);
}

}  // namespace mm
