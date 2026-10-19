#pragma once

#include <cstddef>
#include <cstdint>

namespace mm {

/**
 * HAL de armazenamento local (fallback offline) da sessão de navegação - HU-11.
 * Uso: begin() -> write() uma ou mais vezes -> commit() (ou abort() se falhar).
 * A implementação deve garantir que um arquivo só "existe" depois do commit(),
 * para que uma queda de energia não deixe JSON corrompido.
 */
class ISessionStorage {
public:
    virtual ~ISessionStorage() = default;

    virtual bool begin(uint32_t session_id) = 0;
    virtual bool write(const char *chunk, size_t len) = 0;
    virtual bool commit() = 0;
    virtual void abort() = 0;
};

}  // namespace mm
