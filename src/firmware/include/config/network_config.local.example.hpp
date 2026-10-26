#pragma once

// Copie para network_config.local.hpp (arquivo ignorado pelo Git) e personalize.
// Use o IP da máquina que hospeda o backend, não localhost/127.0.0.1.
#define MM_WIFI_SSID "NOME_DA_REDE_WIFI"
#define MM_WIFI_PASSWORD "SENHA_DA_REDE_WIFI"
#define MM_TELEMETRY_HOST "192.168.1.100"
#define MM_TELEMETRY_PORT 3000
#define MM_TELEMETRY_PATH "/ws?role=device"

// Largura em colunas; a altura é 4. Canto: 0=superior esquerdo,
// 1=superior direito, 2=inferior esquerdo, 3=inferior direito.
// Orientação inicial: 0=Norte, 1=Leste, 2=Sul, 3=Oeste.
#define MM_MAZE_WIDTH 4
#define MM_START_CORNER 0
#define MM_INITIAL_HEADING 1

// Calibre os valores de acordo com o resistor shunt e o intervalo do seu módulo.
#define MM_INA226_SHUNT_RESISTANCE_OHMS 0.01f
#define MM_INA226_MAX_EXPECTED_CURRENT_A 5.0f
