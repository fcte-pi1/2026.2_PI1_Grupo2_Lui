// TODO: testar comportamento os ganhos, o tratamento de saturação e a resposta do controlador nas condições reais.

#include "pid_controller.hpp"

#include <math.h>

namespace
{
    float clampValue(float value, float low, float high)
    {
        if (value < low)
            return low;
        if (value > high)
            return high;
        return value;
    }
}

PIDController::PIDController(
    float kp, // kp Corrige o erro atual.
    float ki, // ki Compensa erros persistentes ao longo do tempo.
    float kd, // kd Reage às variações do erro, ajudando a reduzir oscilações
    float minOutput, float maxOutput)
    : kp_(kp), ki_(ki), kd_(kd),
      minOutput_(minOutput), maxOutput_(maxOutput),
      integral_(0.0f), previousMeasurement_(0.0f), firstUpdate_(true)
{
}

float PIDController::update(float setpoint, float measurement,
                            float dtSeconds)
{
    if (dtSeconds <= 0.0f || !isfinite(dtSeconds) ||
        !isfinite(setpoint) || !isfinite(measurement))
    {
        return 0.0f;
    }

    const float error = setpoint - measurement;
    float derivative = 0.0f;
    if (!firstUpdate_)
    {
        // Derivada da medida para reduzir o derivative kick ao trocar setpoint.
        derivative = (measurement - previousMeasurement_) / dtSeconds;
    }

    const float candidateIntegral = integral_ + error * dtSeconds;
    const float p = kp_ * error;
    const float d = -kd_ * derivative;
    const float candidateOutput = p + ki_ * candidateIntegral + d;

    // Anti-windup condicional: não integra mais quando a saída satura e o erro
    // empurra ainda mais na direção da saturação.
    const bool pushesHigh = candidateOutput > maxOutput_ && error > 0.0f;
    const bool pushesLow = candidateOutput < minOutput_ && error < 0.0f;
    if (!pushesHigh && !pushesLow)
    {
        integral_ = candidateIntegral;
    }

    previousMeasurement_ = measurement;
    firstUpdate_ = false;

    const float output = p + ki_ * integral_ + d;
    return clampValue(output, minOutput_, maxOutput_);
}

void PIDController::reset()
{
    integral_ = 0.0f;
    previousMeasurement_ = 0.0f;
    firstUpdate_ = true;
}