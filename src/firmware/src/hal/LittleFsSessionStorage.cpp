#ifdef ARDUINO

#include "hal/LittleFsSessionStorage.h"

#include <LittleFS.h>

#include <cinttypes>
#include <cstdio>

namespace mm {

bool LittleFsSessionStorage::mount(bool format_on_fail)
{
    if (!LittleFS.begin(format_on_fail)) {
        return false;
    }
    if (!LittleFS.exists("/sessions")) {
        return LittleFS.mkdir("/sessions");
    }
    return true;
}

bool LittleFsSessionStorage::begin(uint32_t session_id)
{
    if (open_) {
        abort();
    }
    std::snprintf(final_path_, sizeof(final_path_), "/sessions/%" PRIu32 ".json", session_id);
    std::snprintf(tmp_path_, sizeof(tmp_path_), "/sessions/%" PRIu32 ".json.tmp", session_id);

    file_ = LittleFS.open(tmp_path_, FILE_WRITE);
    open_ = static_cast<bool>(file_);
    return open_;
}

bool LittleFsSessionStorage::write(const char *chunk, size_t len)
{
    return open_ && file_.write(reinterpret_cast<const uint8_t *>(chunk), len) == len;
}

bool LittleFsSessionStorage::commit()
{
    if (!open_) {
        return false;
    }
    file_.close();
    open_ = false;
    LittleFS.remove(final_path_);                 /* rename não sobrescreve em todas as versões */
    return LittleFS.rename(tmp_path_, final_path_);
}

void LittleFsSessionStorage::abort()
{
    if (open_) {
        file_.close();
        open_ = false;
    }
    LittleFS.remove(tmp_path_);
}

}  // namespace mm

#endif  // ARDUINO
