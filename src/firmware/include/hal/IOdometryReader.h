#pragma once

#include <cstdint>

namespace tracking {

// Leitura bruta de deslocamento linear acumulado desde o último reset do
// acumulador, já convertida para milímetros pela implementação concreta.
struct OdometryReading {
    uint16_t accumulatedDistanceMm;
    uint32_t timestampMs;
};

// Fronteira de HAL entre a lógica de rastreamento de posição e o hardware
// de odometria. Implementada futuramente por um driver real dos encoders
// N20 (fora do escopo desta feature); nesta feature existe apenas um duplo
// de teste (em test/).
class IOdometryReader {
public:
    virtual ~IOdometryReader() = default;

    // Lê o deslocamento acumulado desde o último reset. Não deve bloquear
    // além do orçamento do ciclo de controle do chamador.
    virtual OdometryReading read() = 0;

    // Zera o acumulador de distância (chamado após confirmar um avanço de
    // célula), sem afetar nenhum outro estado do sensor.
    virtual void resetAccumulator() = 0;
};

}  // namespace tracking
