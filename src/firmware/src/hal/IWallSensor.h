#pragma once

#include <cstdint>

// NOTA: este arquivo fica em include/hal/ (Constitution/SETUP.md — camada de
// Drivers de Hardware), mas o namespace `perception` é mantido por ser o
// agrupamento lógico da feature HU-01 (percepção de paredes), que abrange
// tanto esta interface de HAL quanto a lógica em include/navigation/
// WallPerception.h. Pasta física e namespace lógico não precisam coincidir.
namespace perception {

// Uma das três direções relativas ao robô. NÃO é uma direção absoluta do
// labirinto (norte/sul/leste/oeste) — ver documentacao/hu-01-deteccao-medicao-paredes.md.
enum class Direction : uint8_t {
    Front,
    Left,
    Right
};

// Leitura bruta de um sensor, já convertida para milímetros pela
// implementação concreta de IWallSensor (o driver real é responsável pela
// conversão de tempo de pulso para distância — ver
// documentacao/hu-01-deteccao-medicao-paredes.md).
struct RawEcho {
    uint16_t rawDistanceMm;
    bool hasEcho;       // false = sem eco (fora de alcance); NÃO é uma leitura inválida.
    uint32_t timestampMs;
};

// Fronteira de HAL entre a lógica de percepção e o hardware. Implementada
// futuramente por um driver real do US-100 (Us100WallSensor, fora do
// escopo de HU-01); nesta feature existe apenas MockWallSensor (em test/).
class IWallSensor {
public:
    virtual ~IWallSensor() = default;

    // Dispara e lê UM sensor. Não deve bloquear além do orçamento do ciclo
    // de 10ms (FR-008) do ponto de vista do chamador.
    virtual RawEcho read(Direction direction) = 0;
};

}  // namespace perception
