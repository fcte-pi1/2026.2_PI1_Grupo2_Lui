#pragma once

// Este cabeçalho deve ser incluído somente pelos testes de componentes I2C.
// Mantê-lo separado evita que testes sem I2C (ex.: US-100) dependam de Wire.h.
#include <Wire.h>
#include <stdint.h>

namespace DiagnosticI2C {

inline bool probe(TwoWire& bus, uint8_t address, uint8_t& status) {
    bus.beginTransmission(address);
    status = bus.endTransmission(true);
    return status == 0;
}

}  // namespace DiagnosticI2C
