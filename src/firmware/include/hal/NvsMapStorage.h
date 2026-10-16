#pragma once

#include "hal/IMapStorage.h"

namespace mm {

/**
 * Implementação de IMapStorage sobre a NVS do ESP32 (namespace "maze", chave "map").
 * Pré-requisito: nvs_flash_init() ter sido chamado no boot.
 * Só é compilada com ESP_PLATFORM (ver NvsMapStorage.cpp); nos testes nativos
 * não é usada.
 */
class NvsMapStorage : public IMapStorage {
public:
    bool   save(const uint8_t *data, size_t len) override;
    size_t load(uint8_t *out, size_t max_len) override;
};

}  // namespace mm
