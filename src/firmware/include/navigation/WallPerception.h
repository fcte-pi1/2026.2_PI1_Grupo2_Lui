#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "hal/IWallSensor.h"

// NOTA: fica em include/navigation/ (Constitution/SETUP.md — camada de
// Mapeamento e Navegação), mas mantém o namespace `perception`, que agrupa
// toda a lógica da HU-01 (ver include/hal/IWallSensor.h para a mesma nota).
namespace perception {

// Limiar configurável de distância para decidir "parede presente" vs.
// "caminho livre" (FR-004/FR-005). Valor placeholder — A CALIBRAR
// FISICAMENTE quando o robô físico existir (ver
// documentacao/hu-01-deteccao-medicao-paredes.md). Referência de projeto:
// corredor padrão de 180mm de largura (RNF04), descontada a envoltória do
// robô. Não depende do tamanho do labirinto (FR-011): o mesmo valor vale
// para os labirintos 4x4, 8x4 e 12x4, já que todos compartilham a mesma
// largura de corredor.
inline constexpr uint16_t kCorridorWallThresholdMm = 90;

// Uma amostra de uma direção em um instante.
struct SensorReading {
    Direction direction = Direction::Front;
    uint16_t rawDistanceMm = 0;
    uint16_t filteredDistanceMm = 0;
    uint32_t timestampMs = 0;
    bool valid = true;
};

// Faixa física plausível do sensor ultrassônico US-100 (ver datasheet,
// docs/4.3). Usada apenas para descartar leituras fisicamente impossíveis
// (FR-002 / Edge Cases) — não confundir com kCorridorWallThresholdMm.
inline constexpr uint16_t kUs100MinRangeMm = 20;    // ~2cm
inline constexpr uint16_t kUs100MaxRangeMm = 4500;  // ~450cm

// true quando a leitura bruta está dentro da faixa física plausível do
// sensor. Não considera ausência de eco (isso é tratado separadamente em
// applyReading — ausência de eco NÃO é uma leitura inválida).
bool isPhysicallyPlausible(uint16_t rawDistanceMm);

// Suavização por média móvel de janela fixa (N = 3 amostras), uma
// instância por direção.
class MovingAverageFilter3 {
public:
    uint16_t push(uint16_t rawValue);

private:
    static constexpr uint8_t kWindowSize = 3;
    std::array<uint16_t, kWindowSize> buffer_{};
    uint8_t cursor_ = 0;
    uint8_t filledCount_ = 0;
};

// Atualiza `reading` a partir de uma nova leitura bruta `echo`, aplicando a
// validação de FR-002/Edge Cases e a suavização de FR-003 via `filter`
// (uma instância de MovingAverageFilter3 por direção, mantida pelo
// chamador). Ausência de eco (`echo.hasEcho == false`) é tratada como
// "caminho livre" (valor no alcance máximo do sensor), nunca como leitura
// inválida; só uma leitura com eco porém fisicamente impossível marca
// `reading.valid = false`, e nesse caso o filtro NÃO é alimentado.
void applyReading(SensorReading& reading, const RawEcho& echo, MovingAverageFilter3& filter);

// Condição de falha de uma direção específica. Consumida apenas por HU-12
// (diagnóstico) — nunca influencia WallCellState.
struct SensorFaultState {
    Direction direction = Direction::Front;
    uint8_t consecutiveInvalidCount = 0;
    bool isFaulted = false;
};

// Máquina de estados de FR-010: 5 leituras inválidas/ausentes consecutivas
// (`readingValid == false`) marcam `isFaulted = true`; 1 leitura válida
// reseta imediatamente para Normal (`isFaulted = false`,
// `consecutiveInvalidCount = 0`).
void updateFaultState(SensorFaultState& fault, bool readingValid);

// Estado consolidado de uma atualização de célula — é o que alimenta
// HU-00 (mapeamento, via
// wallFront/wallLeft/wallRight) e HU-10 (telemetria, via currentState()
// completo). As direções são relativas ao robô, NÃO ao labirinto; a
// tradução para N/S/E/W não é responsabilidade deste módulo (FR-006).
struct WallCellState {
    // Presença de parede derivada da leitura válida mais recente. Quando a
    // leitura é inválida, a flag fica false e o consumidor deve consultar
    // o campo *Valid* para distinguir "sem parede" de "desconhecido".
    bool wallFront = false;
    bool wallLeft = false;
    bool wallRight = false;

    bool frontValid = false;
    bool leftValid = false;
    bool rightValid = false;

    // Distâncias suavizadas em mm. São zero quando a amostra atual é inválida,
    // evitando que consumidores usem inadvertidamente uma distância obsoleta.
    uint16_t distFrontMm = 0;
    uint16_t distLeftMm = 0;
    uint16_t distRightMm = 0;
    uint32_t timestampMs = 0;
};

// Módulo de percepção de paredes de HU-01. Ver
// documentacao/hu-01-deteccao-medicao-paredes.md para o contrato completo
// com os consumidores (HU-00, HU-04/HU-05, HU-10, HU-12).
class WallPerception {
public:
    explicit WallPerception(IWallSensor& sensor, uint16_t thresholdMm = kCorridorWallThresholdMm);

    // Lê as 3 direções, aplica validação+suavização e deriva
    // wallFront/wallLeft/wallRight (FR-004/FR-005), atualizando também o
    // estado de falha por direção (FR-010). Chamar a cada ciclo de
    // Task_Sensors (10ms, FR-008).
    void update();

    // Snapshot mais recente — só é significativo após ao menos uma chamada
    // a update(). Consumido por HU-00 (mapeamento) e HU-10 (telemetria).
    // Contrato (documentacao/hu-01-deteccao-medicao-paredes.md): os valores
    // retornados aqui já estão prontos (validados + suavizados por
    // update()); HU-10 NÃO deve reprocessar/suavizar novamente antes de
    // serializar para telemetria.
    const WallCellState& currentState() const { return state_; }

    // Consumido por HU-04/HU-05 (alinhamento/PID) a cada ciclo de
    // controle. `side` deve ser Direction::Left ou Direction::Right.
    uint16_t lateralDistanceMm(Direction side) const;

    // Consumido apenas por HU-12 (diagnóstico). Nunca influencia
    // currentState() nem lateralDistanceMm().
    bool hasFault(Direction direction) const;

private:
    static constexpr std::size_t kDirectionCount = 3;
    static std::size_t indexOf(Direction direction) {
        return static_cast<std::size_t>(direction);
    }

    IWallSensor& sensor_;
    uint16_t thresholdMm_;

    std::array<MovingAverageFilter3, kDirectionCount> filters_{};
    std::array<SensorReading, kDirectionCount> readings_{};
    std::array<SensorFaultState, kDirectionCount> faults_{};
    WallCellState state_{};
};

}  // namespace perception
