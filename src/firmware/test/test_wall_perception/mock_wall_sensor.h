#pragma once

#include <array>
#include <cstddef>
#include <deque>

#include "hal/IWallSensor.h"

namespace perception::test {

// Sensor simulado usado apenas nos testes nativos (sem hardware). Cada
// direção tem sua própria fila programável de leituras; read() consome a
// próxima leitura da fila daquela direção. Quando a fila esvazia, repete a
// última leitura enfileirada, para não travar o chamador em testes que leem
// mais vezes do que enfileiraram.
class MockWallSensor : public IWallSensor {
public:
    void enqueue(Direction direction, RawEcho echo) {
        queues_[indexOf(direction)].push_back(echo);
    }

    RawEcho read(Direction direction) override {
        auto& queue = queues_[indexOf(direction)];
        if (queue.empty()) {
            return lastReturned_[indexOf(direction)];
        }
        const RawEcho next = queue.front();
        queue.pop_front();
        lastReturned_[indexOf(direction)] = next;
        return next;
    }

private:
    static constexpr std::size_t indexOf(Direction direction) {
        return static_cast<std::size_t>(direction);
    }

    std::array<std::deque<RawEcho>, 3> queues_{};
    std::array<RawEcho, 3> lastReturned_{};
};

}  // namespace perception::test
