#pragma once

#include "hal/IOrientationSensor.h"

namespace tracking::test {

// Duplo de teste de IOrientationSensor: o ângulo Yaw é programável
// diretamente pelo teste.
class MockOrientationSensor : public IOrientationSensor {
public:
    void setYawDegrees(int16_t value, uint32_t timestampMs) {
        yawDegrees_ = value;
        timestampMs_ = timestampMs;
    }

    OrientationReading read() override {
        return OrientationReading{yawDegrees_, timestampMs_};
    }

private:
    int16_t yawDegrees_ = 0;
    uint32_t timestampMs_ = 0;
};

}  // namespace tracking::test
