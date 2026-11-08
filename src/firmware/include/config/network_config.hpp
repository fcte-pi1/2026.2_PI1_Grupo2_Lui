#pragma once

#include <cstdint>

// Configurações locais ficam fora do controle de versão. Copie
// network_config.local.example.hpp para network_config.local.hpp e personalize
// o nome Bluetooth e os parâmetros físicos do INA226.
#if defined(__has_include)
#  if __has_include("network_config.local.hpp")
#    include "network_config.local.hpp"
#  endif
#endif

#ifndef MM_BT_DEVICE_NAME
#define MM_BT_DEVICE_NAME "MicroMouse"
#endif
#ifndef MM_MAZE_WIDTH
#define MM_MAZE_WIDTH 4
#endif
#ifndef MM_START_CORNER
#define MM_START_CORNER 0
#endif
#ifndef MM_INITIAL_HEADING
#define MM_INITIAL_HEADING 1
#endif
#ifndef MM_INA226_SHUNT_RESISTANCE_OHMS
#define MM_INA226_SHUNT_RESISTANCE_OHMS 0.01f
#endif
#ifndef MM_INA226_MAX_EXPECTED_CURRENT_A
#define MM_INA226_MAX_EXPECTED_CURRENT_A 5.0f
#endif

namespace mm::network {
// Calibre os valores conforme o resistor shunt e a corrente máxima do módulo.
constexpr float INA226_SHUNT_RESISTANCE_OHMS = MM_INA226_SHUNT_RESISTANCE_OHMS;
constexpr float INA226_MAX_EXPECTED_CURRENT_A = MM_INA226_MAX_EXPECTED_CURRENT_A;
constexpr uint32_t TELEMETRY_PERIOD_MS = 250;
constexpr uint32_t POWER_SAMPLE_PERIOD_MS = 500;
constexpr uint32_t TELEMETRY_QUEUE_STALE_MS = 1500;
}  // namespace mm::network
