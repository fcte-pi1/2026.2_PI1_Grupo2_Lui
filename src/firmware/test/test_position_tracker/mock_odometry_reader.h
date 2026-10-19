#pragma once

#include "hal/IOdometryReader.h"

namespace tracking::test {

// Duplo de teste de IOdometryReader: a distância acumulada é programável
// diretamente pelo teste, e resetAccumulator() zera o valor programado,
// imitando o comportamento esperado de um driver real.
class MockOdometryReader : public IOdometryReader {
public:
    void setAccumulatedDistanceMm(uint16_t value, uint32_t timestampMs) {
        distanceMm_ = value;
        timestampMs_ = timestampMs;
    }

    OdometryReading read() override {
        return OdometryReading{distanceMm_, timestampMs_};
    }

    void resetAccumulator() override {
        distanceMm_ = 0;
    }

private:
    uint16_t distanceMm_ = 0;
    uint32_t timestampMs_ = 0;
};

}  // namespace tracking::test
