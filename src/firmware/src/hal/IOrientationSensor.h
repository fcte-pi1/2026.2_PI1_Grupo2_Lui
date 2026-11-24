#pragma once

#include <cstdint>

namespace tracking {

// Leitura bruta do ângulo de orientação absoluto atual (Yaw), em graus
// (0–359), já convertida pela implementação concreta a partir do sensor.
struct OrientationReading {
    int16_t yawDegrees;
    uint32_t timestampMs;
};

// Fronteira de HAL entre a lógica de rastreamento de posição e o hardware
// de orientação (IMU). Implementada futuramente por um driver real (fora
// do escopo desta feature); nesta feature existe apenas um duplo de teste.
class IOrientationSensor {
public:
    virtual ~IOrientationSensor() = default;

    // Lê o ângulo absoluto atual. Não retorna um delta — quem consome é
    // responsável por calcular a variação em relação a uma referência
    // anterior. Não deve bloquear além do orçamento do ciclo do chamador.
    virtual OrientationReading read() = 0;
};

}  // namespace tracking
