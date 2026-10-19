#ifdef ESP_PLATFORM

#include "hal/NvsMapStorage.h"

#include "nvs.h"

namespace mm {

namespace {
constexpr const char *kNamespace = "maze";
constexpr const char *kKey       = "map";
}  // namespace

bool NvsMapStorage::save(const uint8_t *data, size_t len)
{
    nvs_handle_t h;
    if (nvs_open(kNamespace, NVS_READWRITE, &h) != ESP_OK) {
        return false;
    }
    esp_err_t err = nvs_set_blob(h, kKey, data, len);
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);
    return err == ESP_OK;
}

size_t NvsMapStorage::load(uint8_t *out, size_t max_len)
{
    nvs_handle_t h;
    if (nvs_open(kNamespace, NVS_READONLY, &h) != ESP_OK) {
        return 0;
    }
    size_t len = max_len;
    const esp_err_t err = nvs_get_blob(h, kKey, out, &len);
    nvs_close(h);
    return err == ESP_OK ? len : 0;
}

}  // namespace mm

#endif  // ESP_PLATFORM
