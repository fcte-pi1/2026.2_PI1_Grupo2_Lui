#pragma once

#include <cstddef>
#include <cstdint>

namespace mm {

/**
 * HAL de armazenamento não-volátil do mapa (HU-00).
 * Implementações: NvsMapStorage (ESP32) e um fake em memória nos testes.
 */
class IMapStorage {
public:
    virtual ~IMapStorage() = default;

    /** Grava o blob de forma atômica. true se ok. */
    virtual bool save(const uint8_t *data, size_t len) = 0;

    /** Lê até max_len bytes. Retorna os bytes lidos, ou 0 se não houver mapa salvo. */
    virtual size_t load(uint8_t *out, size_t max_len) = 0;
};

}  // namespace mm
