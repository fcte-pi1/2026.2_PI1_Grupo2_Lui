#pragma once

#include <cstdint>

// Overrides locais ficam fora do controle de versão. Copie
// network_config.local.example.hpp para network_config.local.hpp e configure
// Wi-Fi, endereço do backend e parâmetros físicos do INA226.
#if defined(__has_include)
#  if __has_include("network_config.local.hpp")
#    include "network_config.local.hpp"
#  endif
#endif

// O host deve ser o IP LAN do computador que executa o backend; "localhost"
// apontaria para o próprio ESP32 e não funcionaria.
#ifndef MM_WIFI_SSID
#define MM_WIFI_SSID ""
#endif
#ifndef MM_WIFI_PASSWORD
#define MM_WIFI_PASSWORD ""
#endif
#ifndef MM_TELEMETRY_HOST
#define MM_TELEMETRY_HOST "192.168.1.100"
#endif
#ifndef MM_TELEMETRY_PORT
#define MM_TELEMETRY_PORT 3000
#endif
#ifndef MM_TELEMETRY_PATH
#define MM_TELEMETRY_PATH "/ws?role=device"
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
