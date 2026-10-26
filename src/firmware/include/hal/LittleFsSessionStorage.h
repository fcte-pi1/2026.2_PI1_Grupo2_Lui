#pragma once

/**
 * Implementação de ISessionStorage sobre LittleFS (Arduino-ESP32).
 * Grava em /sessions/<id>.json.tmp e faz rename para /sessions/<id>.json no commit().
 * Só existe com ARDUINO definido: no ambiente [env:native] este header e o .cpp
 * ficam vazios.
 */
#ifdef ARDUINO

#include <FS.h>

#include "hal/ISessionStorage.h"

namespace mm {

class LittleFsSessionStorage : public ISessionStorage {
public:
    /** Montar uma vez no setup(). format_on_fail=true formata se a partição estiver vazia. */
    static bool mount(bool format_on_fail = true);

    bool begin(uint32_t session_id) override;
    bool write(const char *chunk, size_t len) override;
    bool commit() override;
    void abort() override;

private:
    fs::File file_;
    char     tmp_path_[40] = {0};
    char     final_path_[40] = {0};
    bool     open_ = false;
};

}  // namespace mm

#endif  // ARDUINO
