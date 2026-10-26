#include "navigation/SessionCoordinator.h"

namespace mm {

bool SessionCoordinator::begin(uint32_t sessionId, const SessionPose &initial,
                               uint32_t nowMs, const char *tipoLabirinto, uint16_t tentativa)
{
    if (!recorder_.begin(sessionId, initial, nowMs, tipoLabirinto, tentativa)) {
        return false;
    }
    // O mesmo ID liga os logs ao registro de sessão criado no backend.
    logger_.setSessionId(sessionId);
    (void)logger_.info("Sessao iniciada", 0);
    return true;
}

bool SessionCoordinator::finish(uint32_t nowMs, EndReason reason)
{
    if (!recorder_.end(nowMs, reason)) {
        return false;
    }

    const uint32_t timestamp = buffer_.totalTimeMs();
    switch (reason) {
        case EndReason::FinishLine:
            (void)logger_.success("Linha de chegada reconhecida", timestamp);
            break;
        case EndReason::ManualStop:
            (void)logger_.warn("Sessao interrompida manualmente", timestamp);
            break;
        case EndReason::Error:
            (void)logger_.error("Sessao interrompida por falha", timestamp);
            break;
    }
    return true;
}

}  // namespace mm
