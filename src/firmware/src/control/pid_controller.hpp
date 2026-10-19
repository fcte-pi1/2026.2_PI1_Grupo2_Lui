#pragma once

class PIDController {
public:
    PIDController(float kp, float ki, float kd,
                  float minOutput, float maxOutput);

    float update(float setpoint, float measurement, float dtSeconds);
    void reset();

private:
    float kp_;
    float ki_;
    float kd_;
    float minOutput_;
    float maxOutput_;
    float integral_;
    float previousMeasurement_;
    bool firstUpdate_;
};