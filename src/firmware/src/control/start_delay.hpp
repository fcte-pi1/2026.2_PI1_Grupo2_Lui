#pragma once
#include <cstdint>
// Non-blocking, cancellable delay. The mission's 180s start after it expires.
class StartDelay {
public:
    void arm(uint32_t now, uint32_t delay) { started_ = now; delay_ = delay; pending_ = true; }
    void cancel() { pending_ = false; }
    bool pending() const { return pending_; }
    bool consumeIfDue(uint32_t now) {
        if (!pending_ || static_cast<uint32_t>(now - started_) < delay_) return false;
        pending_ = false;
        return true;
    }
private:
    uint32_t started_ = 0, delay_ = 0;
    bool pending_ = false;
};
