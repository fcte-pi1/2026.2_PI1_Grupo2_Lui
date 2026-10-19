#include "encoder_n20.hpp"

#include "esp_err.h"

bool EncoderN20::begin(uint8_t channelAPin,
                       uint8_t channelBPin,
                       bool invertDirection,
                       uint32_t glitchFilterNs) {
    if (_initialized || channelAPin == channelBPin) {
        return false;
    }

    pinMode(channelAPin, INPUT);
    pinMode(channelBPin, INPUT);

    pcnt_unit_config_t unitConfig = {};
    unitConfig.high_limit = 30000;
    unitConfig.low_limit = -30000;
    unitConfig.flags.accum_count = 1;

    if (pcnt_new_unit(&unitConfig, &_unit) != ESP_OK) {
        _unit = nullptr;
        return false;
    }

    pcnt_glitch_filter_config_t filterConfig = {};
    filterConfig.max_glitch_ns = glitchFilterNs;
    if (pcnt_unit_set_glitch_filter(_unit, &filterConfig) != ESP_OK) {
        return false;
    }

    pcnt_chan_config_t channelConfig = {};
    channelConfig.edge_gpio_num = channelAPin;
    channelConfig.level_gpio_num = channelBPin;

    if (pcnt_new_channel(_unit, &channelConfig, &_channel) != ESP_OK) {
        return false;
    }

    const pcnt_channel_edge_action_t positiveEdge = invertDirection
        ? PCNT_CHANNEL_EDGE_ACTION_DECREASE
        : PCNT_CHANNEL_EDGE_ACTION_INCREASE;
    const pcnt_channel_edge_action_t negativeEdge = invertDirection
        ? PCNT_CHANNEL_EDGE_ACTION_INCREASE
        : PCNT_CHANNEL_EDGE_ACTION_DECREASE;

    if (pcnt_channel_set_edge_action(
            _channel, positiveEdge, negativeEdge) != ESP_OK) {
        return false;
    }

    if (pcnt_channel_set_level_action(
            _channel,
            PCNT_CHANNEL_LEVEL_ACTION_KEEP,
            PCNT_CHANNEL_LEVEL_ACTION_INVERSE) != ESP_OK) {
        return false;
    }

    // Watch points são necessários para acumular corretamente os overflows
    // do contador de hardware em contagens de 32 bits.
    if (pcnt_unit_add_watch_point(_unit, 30000) != ESP_OK ||
        pcnt_unit_add_watch_point(_unit, -30000) != ESP_OK) {
        return false;
    }

    if (pcnt_unit_enable(_unit) != ESP_OK) {
        return false;
    }
    if (pcnt_unit_clear_count(_unit) != ESP_OK) {
        return false;
    }
    if (pcnt_unit_start(_unit) != ESP_OK) {
        return false;
    }

    _initialized = true;
    return true;
}

bool EncoderN20::readCount(int32_t& count) const {
    if (!_initialized || _unit == nullptr) {
        return false;
    }
    int hardwareCount = 0;
    if (pcnt_unit_get_count(_unit, &hardwareCount) != ESP_OK) {
        return false;
    }
    count = static_cast<int32_t>(hardwareCount);
    return true;
}

bool EncoderN20::resetCount() {
    if (!_initialized || _unit == nullptr) {
        return false;
    }
    return pcnt_unit_clear_count(_unit) == ESP_OK;
}

bool EncoderN20::isInitialized() const {
    return _initialized;
}
