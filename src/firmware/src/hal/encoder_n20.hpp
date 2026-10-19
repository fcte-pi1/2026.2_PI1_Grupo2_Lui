#pragma once

#include <Arduino.h>
#include <stdint.h>
#include "driver/pulse_cnt.h"

// Encoder em quadratura usando o PCNT do ESP32: canal A como sinal de borda
// e canal B como nível para determinar a direção. Conta as bordas de A;
// confirme a convenção de sinal na montagem real.
class EncoderN20 {
public:
    bool begin(uint8_t channelAPin,
               uint8_t channelBPin,
               bool invertDirection = false,
               uint32_t glitchFilterNs = 1000UL);

    bool readCount(int32_t& count) const;
    bool resetCount();
    bool isInitialized() const;

private:
    pcnt_unit_handle_t _unit = nullptr;
    pcnt_channel_handle_t _channel = nullptr;
    bool _initialized = false;
};
