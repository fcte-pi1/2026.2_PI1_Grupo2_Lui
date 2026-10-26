#pragma once

#include <cstdint>

#include "config/maze_geometry.hpp"
#include "hal/IOdometryReader.h"
#include "hal/IOrientationSensor.h"

// NOTA: fica em include/navigation/ (camada de Mapeamento e Navegação),
// mas mantém o namespace `tracking`, que agrupa toda a lógica desta
// feature (rastreamento de posição), incluindo a fronteira de HAL em
// include/hal/IOdometryReader.h e include/hal/IOrientationSensor.h.
namespace tracking {

// Direção absoluta do robô no referencial do labirinto (Norte/Sul/Leste/
// Oeste) — não deve ser confundida com direção relativa ao robô (frente/
// esquerda/direita), usada por outros módulos de percepção.
enum class Orientation : uint8_t {
    North,
    East,
    South,
    West
};

// Posição cartesiana (linha, coluna) do robô na grade do labirinto.
struct GridPosition {
    int16_t row = 0;
    int16_t col = 0;
};

// Estado consolidado de posição + orientação em um dado momento.
struct TrackingState {
    GridPosition position{};
    Orientation orientation = Orientation::North;
    uint32_t timestampMs = 0;
};

// Gira `current` `steps` posições no ciclo Norte->Leste->Sul->Oeste->Norte
// (sentido horário quando `clockwise == true`, anti-horário caso
// contrário). `steps == 1` corresponde a 90 graus, `steps == 2` a 180.
Orientation rotateOrientation(Orientation current, uint8_t steps, bool clockwise);

// Aplica o avanço de uma célula a `current`, na direção de `orientation`.
// Convenção de eixos: Norte diminui `row`, Sul aumenta `row`, Leste aumenta
// `col`, Oeste diminui `col`. Altera sempre exatamente um dos dois campos.
GridPosition applyCellAdvance(GridPosition current, Orientation orientation);

// Tolerância angular para confirmar uma rotação de 90/180 graus a partir
// de uma leitura real de IMU. Valor placeholder — A CALIBRAR FISICAMENTE
// quando o robô físico existir.
inline constexpr uint8_t kRotationToleranceDegrees = 15;

// Verifica se `yawDeltaDegrees` (variação de ângulo desde a referência do
// início do movimento atual) corresponde a uma rotação de 90 ou 180 graus,
// dentro de kRotationToleranceDegrees. Se confirmado, preenche `outSteps`
// (1 para 90, 2 para 180) e `outClockwise` (sinal da variação) e retorna
// true; caso contrário retorna false e não altera as saídas.
bool tryConfirmRotation(int16_t yawDeltaDegrees, uint8_t& outSteps, bool& outClockwise);

// Distância acumulada equivalente a uma célula inteira do labirinto.
// Valor placeholder — A CALIBRAR FISICAMENTE quando o robô físico existir.
// Não depende do tamanho do labirinto: o mesmo valor vale para os
// labirintos 4x4, 8x4 e 12x4, já que todos compartilham a mesma largura de
// corredor.
// 75% da célula: permite confirmar o deslocamento mesmo quando o controlador
// desacelera e conclui dentro da tolerância posicional definida na HU-05.
inline constexpr int16_t kCellDistanceThresholdMm =
    static_cast<int16_t>(MazeSettings::CELL_SIZE_MM * 0.75f);

// Intervalo mínimo entre dois avanços de célula aceitos consecutivamente.
// Um novo avanço que chegue antes desse intervalo desde o último aceito é
// ignorado (proteção contra leitura duplicada/ruído). Valor placeholder —
// a calibrar/ajustar quando o hardware existir.
inline constexpr uint32_t kMinCellAdvanceIntervalMs = 200;

// Rastreia a posição (linha/coluna) e a orientação absoluta do robô na
// grade do labirinto, a partir de leituras de odometria e orientação. Ver
// a documentação técnica do módulo para o contrato completo com os
// consumidores (mapeamento, navegação, telemetria).
class PositionTracker {
public:
    // Inicialização padrão para compatibilidade: origem (0,0), orientação Norte.
    explicit PositionTracker(IOdometryReader& odometry, IOrientationSensor& orientationSensor);

    // Permite iniciar em qualquer canto/orientação do labirinto. O yaw informado
    // é a referência inicial do sensor IMU, não uma direção cardinal.
    // Útil quando a configuração da missão define uma origem diferente de (0,0).
    PositionTracker(IOdometryReader& odometry,
                    IOrientationSensor& orientationSensor,
                    const TrackingState& initialState,
                    int16_t initialYawReferenceDegrees = 0);

    // Lê odometria e orientação, confirma avanços/rotações e recalcula o
    // estado. Chamar a cada ciclo de controle relevante.
    void update();

    // Reinicializa a referência de grade no início de cada missão, sem criar
    // uma segunda estimativa de pose no controlador de navegação.
    void reset(const TrackingState& initialState, int16_t initialYawReferenceDegrees);

    // Snapshot mais recente — significativo desde a inicialização, mesmo
    // antes da primeira chamada a update(). Somente leitura: não modifica
    // nenhum estado interno e nunca é retornado parcialmente preenchido.
    const TrackingState& currentState() const { return state_; }

private:
    IOdometryReader& odometry_;
    IOrientationSensor& orientationSensor_;

    TrackingState state_{};
    bool hasAcceptedAdvance_ = false;
    uint32_t lastAdvanceAcceptedMs_ = 0;

    int16_t movementYawReferenceDegrees_ = 0;
};

}  // namespace tracking
