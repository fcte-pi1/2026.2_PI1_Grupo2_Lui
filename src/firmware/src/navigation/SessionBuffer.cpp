#include "navigation/SessionBuffer.h"

#include <cinttypes>
#include <cstdio>

namespace mm {

void SessionBuffer::reset()
{
    state_         = SessionState::Idle;
    reason_        = EndReason::FinishLine;
    session_id_    = 0;
    start_ms_      = 0;
    end_ms_        = 0;
    total_time_ms_ = 0;
    stride_        = 1;
    seen_          = 0;
    count_         = 0;
}

bool SessionBuffer::start(uint32_t session_id, uint32_t now_ms)
{
    if (state_ == SessionState::Running) {
        return false;
    }
    reset();
    session_id_ = session_id;
    start_ms_   = now_ms;
    state_      = SessionState::Running;
    return true;
}

/* Mantém os pontos de índice par e dobra o stride. */
void SessionBuffer::compact()
{
    size_t j = 0;
    for (size_t i = 0; i < count_; i += 2) {
        points_[j++] = points_[i];
    }
    count_   = j;
    stride_ *= 2;
}

bool SessionBuffer::record(uint32_t now_ms, int16_t x, int16_t y, uint8_t heading)
{
    if (state_ != SessionState::Running) {
        return false;
    }

    const uint32_t idx = seen_++;
    if (idx % stride_ != 0) {
        return true;                       /* pulada pela decimação */
    }
    if (count_ == kCapacity) {
        compact();
        if (idx % stride_ != 0) {
            return true;
        }
    }

    RoutePoint &p = points_[count_++];
    p.t_ms    = now_ms - start_ms_;        /* seguro contra overflow de uint32 */
    p.x       = x;
    p.y       = y;
    p.heading = static_cast<uint8_t>(heading & 0x3);
    return true;
}

bool SessionBuffer::stop(uint32_t now_ms, EndReason reason)
{
    if (state_ != SessionState::Running) {
        return false;
    }
    end_ms_        = now_ms;
    total_time_ms_ = now_ms - start_ms_;   /* TempoFinal - TempoInicial */
    reason_        = reason;
    state_         = SessionState::Closed;
    return true;
}

const char *SessionBuffer::reasonStr(EndReason r)
{
    switch (r) {
    case EndReason::FinishLine: return "CHEGADA";
    case EndReason::ManualStop: return "PARADA_MANUAL";
    case EndReason::Error:      return "ERRO";
    }
    return "DESCONHECIDO";
}

namespace {
const char *headingStr(uint8_t h)
{
    static const char *const names[4] = { "NORTH", "EAST", "SOUTH", "WEST" };
    return names[h & 0x3];
}
}  // namespace

bool SessionBuffer::writeJson(WriteFn write, void *ctx) const
{
    if (state_ != SessionState::Closed || write == nullptr) {
        return false;
    }

    char line[160];
    int n = std::snprintf(line, sizeof(line),
        "{\"session_id\":%" PRIu32 ",\"tempo_total_ms\":%" PRIu32
        ",\"motivo\":\"%s\",\"stride\":%" PRIu32 ",\"pontos\":[",
        session_id_, total_time_ms_, reasonStr(reason_), stride_);
    if (n < 0 || static_cast<size_t>(n) >= sizeof(line) ||
        write(line, static_cast<size_t>(n), ctx) != 0) {
        return false;
    }

    for (size_t i = 0; i < count_; i++) {
        const RoutePoint &p = points_[i];
        n = std::snprintf(line, sizeof(line),
            "%s{\"t\":%" PRIu32 ",\"x\":%d,\"y\":%d,\"heading\":\"%s\"}",
            i ? "," : "", p.t_ms, p.x, p.y, headingStr(p.heading));
        if (n < 0 || static_cast<size_t>(n) >= sizeof(line) ||
            write(line, static_cast<size_t>(n), ctx) != 0) {
            return false;
        }
    }
    return write("]}", 2, ctx) == 0;
}

namespace {
int storageWrite(const char *chunk, size_t len, void *ctx)
{
    return static_cast<ISessionStorage *>(ctx)->write(chunk, len) ? 0 : -1;
}
}  // namespace

bool SessionBuffer::saveFallback(ISessionStorage &storage) const
{
    if (state_ != SessionState::Closed || !storage.begin(session_id_)) {
        return false;
    }
    if (!writeJson(storageWrite, &storage)) {
        storage.abort();
        return false;
    }
    if (!storage.commit()) {
        storage.abort();
        return false;
    }
    return true;
}

}  // namespace mm
