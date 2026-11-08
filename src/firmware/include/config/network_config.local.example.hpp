#pragma once

// Copie para network_config.local.hpp (arquivo ignorado pelo Git).
// O backend deve parear com este nome Bluetooth Classic (SPP).
#define MM_BT_DEVICE_NAME "MicroMouse"

// Largura em colunas; a altura é 4. Canto: 0=superior esquerdo,
// 1=superior direito, 2=inferior esquerdo, 3=inferior direito.
// Orientação inicial: 0=Norte, 1=Leste, 2=Sul, 3=Oeste.
#define MM_MAZE_WIDTH 4
#define MM_START_CORNER 0
#define MM_INITIAL_HEADING 1

// Calibre os valores de acordo com o resistor shunt e o intervalo do seu módulo.
#define MM_INA226_SHUNT_RESISTANCE_OHMS 0.01f
#define MM_INA226_MAX_EXPECTED_CURRENT_A 5.0f
