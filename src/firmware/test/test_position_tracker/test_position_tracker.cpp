#include <unity.h>

#include <chrono>

#include "mock_odometry_reader.h"
#include "mock_orientation_sensor.h"
#include "navigation/PositionTracker.h"

using tracking::GridPosition;
using tracking::kCellDistanceThresholdMm;
using tracking::kMinCellAdvanceIntervalMs;
using tracking::OdometryReading;
using tracking::Orientation;
using tracking::OrientationReading;
using tracking::PositionTracker;
using tracking::test::MockOdometryReader;
using tracking::test::MockOrientationSensor;

void setUp(void) {}
void tearDown(void) {}

// Teste de sanidade: confirma que o toolchain host + mocks + interfaces de
// HAL funcionam, antes de qualquer lógica de rastreamento existir.
static void test_mock_odometry_and_orientation_return_programmed_values(void) {
    MockOdometryReader odometry;
    odometry.setAccumulatedDistanceMm(42, 1000);
    const OdometryReading odoReading = odometry.read();
    TEST_ASSERT_EQUAL_UINT16(42, odoReading.accumulatedDistanceMm);
    TEST_ASSERT_EQUAL_UINT32(1000, odoReading.timestampMs);

    MockOrientationSensor orientationSensor;
    orientationSensor.setYawDegrees(90, 2000);
    const OrientationReading yawReading = orientationSensor.read();
    TEST_ASSERT_EQUAL_INT16(90, yawReading.yawDegrees);
    TEST_ASSERT_EQUAL_UINT32(2000, yawReading.timestampMs);
}

// Tabela de transição de Orientation: girar N passos no sentido horário
// avança N posições no ciclo Norte->Leste->Sul->Oeste->Norte; sentido
// anti-horário retrocede; 180 graus equivale a 2 passos.
static void test_rotate_orientation_clockwise_90_cycles_through_all_four(void) {
    TEST_ASSERT_EQUAL(Orientation::East, tracking::rotateOrientation(Orientation::North, 1, true));
    TEST_ASSERT_EQUAL(Orientation::South, tracking::rotateOrientation(Orientation::East, 1, true));
    TEST_ASSERT_EQUAL(Orientation::West, tracking::rotateOrientation(Orientation::South, 1, true));
    TEST_ASSERT_EQUAL(Orientation::North, tracking::rotateOrientation(Orientation::West, 1, true));
}

static void test_rotate_orientation_counter_clockwise_90_reverses_the_cycle(void) {
    TEST_ASSERT_EQUAL(Orientation::West, tracking::rotateOrientation(Orientation::North, 1, false));
    TEST_ASSERT_EQUAL(Orientation::South, tracking::rotateOrientation(Orientation::West, 1, false));
    TEST_ASSERT_EQUAL(Orientation::East, tracking::rotateOrientation(Orientation::South, 1, false));
    TEST_ASSERT_EQUAL(Orientation::North, tracking::rotateOrientation(Orientation::East, 1, false));
}

static void test_rotate_orientation_180_is_the_opposite_direction(void) {
    TEST_ASSERT_EQUAL(Orientation::South, tracking::rotateOrientation(Orientation::North, 2, true));
    TEST_ASSERT_EQUAL(Orientation::North, tracking::rotateOrientation(Orientation::South, 2, true));
    TEST_ASSERT_EQUAL(Orientation::West, tracking::rotateOrientation(Orientation::East, 2, false));
    TEST_ASSERT_EQUAL(Orientation::East, tracking::rotateOrientation(Orientation::West, 2, false));
}

// Um avanço de célula altera row OU col (nunca ambos), no sentido correto
// para a orientação vigente no momento do avanço.
static void test_apply_cell_advance_moves_one_step_per_orientation(void) {
    const GridPosition origin{5, 5};

    const GridPosition north = tracking::applyCellAdvance(origin, Orientation::North);
    TEST_ASSERT_EQUAL_INT16(4, north.row);
    TEST_ASSERT_EQUAL_INT16(5, north.col);

    const GridPosition south = tracking::applyCellAdvance(origin, Orientation::South);
    TEST_ASSERT_EQUAL_INT16(6, south.row);
    TEST_ASSERT_EQUAL_INT16(5, south.col);

    const GridPosition east = tracking::applyCellAdvance(origin, Orientation::East);
    TEST_ASSERT_EQUAL_INT16(5, east.row);
    TEST_ASSERT_EQUAL_INT16(6, east.col);

    const GridPosition west = tracking::applyCellAdvance(origin, Orientation::West);
    TEST_ASSERT_EQUAL_INT16(5, west.row);
    TEST_ASSERT_EQUAL_INT16(4, west.col);
}

// tryConfirmRotation(): uma variação de ~90 graus confirma 1 passo; ~180
// graus confirma 2 passos; abaixo da tolerância de 90 não confirma nada; o
// sinal da variação determina o sentido.
static void test_try_confirm_rotation_90_degrees_within_tolerance(void) {
    uint8_t steps = 0;
    bool clockwise = false;
    const bool confirmed = tracking::tryConfirmRotation(80, steps, clockwise);
    TEST_ASSERT_TRUE(confirmed);
    TEST_ASSERT_EQUAL_UINT8(1, steps);
    TEST_ASSERT_TRUE(clockwise);
}

static void test_try_confirm_rotation_180_degrees_within_tolerance(void) {
    uint8_t steps = 0;
    bool clockwise = true;
    const bool confirmed = tracking::tryConfirmRotation(-170, steps, clockwise);
    TEST_ASSERT_TRUE(confirmed);
    TEST_ASSERT_EQUAL_UINT8(2, steps);
    TEST_ASSERT_FALSE(clockwise);
}

static void test_try_confirm_rotation_below_threshold_does_not_confirm(void) {
    uint8_t steps = 0;
    bool clockwise = false;
    const bool confirmed = tracking::tryConfirmRotation(30, steps, clockwise);
    TEST_ASSERT_FALSE(confirmed);
}

// User Story 1 — avanço de célula completo muda a posição corretamente,
// na orientação inicial padrão (Norte).
static void test_position_tracker_confirms_advance_and_updates_position(void) {
    MockOdometryReader odometry;
    MockOrientationSensor orientationSensor;
    PositionTracker tracker(odometry, orientationSensor);

    odometry.setAccumulatedDistanceMm(kCellDistanceThresholdMm, 100);
    tracker.update();

    TEST_ASSERT_EQUAL_INT16(-1, tracker.currentState().position.row);
    TEST_ASSERT_EQUAL_INT16(0, tracker.currentState().position.col);
}

// Deslocamento abaixo do limiar não muda a posição.
static void test_position_tracker_below_threshold_does_not_change_position(void) {
    MockOdometryReader odometry;
    MockOrientationSensor orientationSensor;
    PositionTracker tracker(odometry, orientationSensor);

    odometry.setAccumulatedDistanceMm(static_cast<uint16_t>(kCellDistanceThresholdMm - 1), 100);
    tracker.update();

    TEST_ASSERT_EQUAL_INT16(0, tracker.currentState().position.row);
    TEST_ASSERT_EQUAL_INT16(0, tracker.currentState().position.col);
}

// Dois avanços confirmados separados por menos que o intervalo mínimo
// aplicam só uma transição de posição (deduplicação).
static void test_position_tracker_deduplicates_rapid_advance_confirmations(void) {
    MockOdometryReader odometry;
    MockOrientationSensor orientationSensor;
    PositionTracker tracker(odometry, orientationSensor);

    odometry.setAccumulatedDistanceMm(kCellDistanceThresholdMm, 100);
    tracker.update();
    TEST_ASSERT_EQUAL_INT16(-1, tracker.currentState().position.row);

    // Segundo avanço confirmado antes do intervalo mínimo desde o último aceito.
    odometry.setAccumulatedDistanceMm(kCellDistanceThresholdMm,
                                       100 + (kMinCellAdvanceIntervalMs / 2));
    tracker.update();

    TEST_ASSERT_EQUAL_INT16(-1, tracker.currentState().position.row);
}

// Independência de tamanho de labirinto: não há nenhum parâmetro
// de tamanho de labirinto em PositionTracker — este teste confirma que o
// mesmo comportamento de avanço se repete identicamente, qualquer que seja
// o "labirinto" em que o teste é executado (4x4, 8x4 ou 12x4 são apenas
// limites externos de grade, nunca uma entrada desta classe).
static void test_position_tracker_advance_behavior_independent_of_maze_size(void) {
    for (int mazeSizeSimulated = 0; mazeSizeSimulated < 3; ++mazeSizeSimulated) {
        MockOdometryReader odometry;
        MockOrientationSensor orientationSensor;
        PositionTracker tracker(odometry, orientationSensor);

        odometry.setAccumulatedDistanceMm(kCellDistanceThresholdMm, 100);
        tracker.update();

        TEST_ASSERT_EQUAL_INT16(-1, tracker.currentState().position.row);
        TEST_ASSERT_EQUAL_INT16(0, tracker.currentState().position.col);
    }
}

// User Story 2 — uma rotação confirmada muda a orientação, e o próximo
// avanço é traduzido usando a nova orientação, não a anterior.
static void test_position_tracker_rotation_updates_orientation_then_next_advance(void) {
    MockOdometryReader odometry;
    MockOrientationSensor orientationSensor;
    PositionTracker tracker(odometry, orientationSensor);

    orientationSensor.setYawDegrees(90, 50);
    tracker.update();
    TEST_ASSERT_EQUAL(Orientation::East, tracker.currentState().orientation);

    odometry.setAccumulatedDistanceMm(kCellDistanceThresholdMm, 150);
    tracker.update();

    TEST_ASSERT_EQUAL_INT16(0, tracker.currentState().position.row);
    TEST_ASSERT_EQUAL_INT16(1, tracker.currentState().position.col);
}

// User Story 3 — o estado é consultável a qualquer momento, inclusive
// antes da primeira chamada a update(), e consultas repetidas não têm
// efeito colateral sobre o processamento de novos eventos.
static void test_position_tracker_current_state_available_before_first_update(void) {
    MockOdometryReader odometry;
    MockOrientationSensor orientationSensor;
    PositionTracker tracker(odometry, orientationSensor);

    TEST_ASSERT_EQUAL_INT16(0, tracker.currentState().position.row);
    TEST_ASSERT_EQUAL_INT16(0, tracker.currentState().position.col);
    TEST_ASSERT_EQUAL(Orientation::North, tracker.currentState().orientation);
}

static void test_position_tracker_repeated_queries_do_not_affect_state(void) {
    MockOdometryReader odometry;
    MockOrientationSensor orientationSensor;
    PositionTracker tracker(odometry, orientationSensor);

    odometry.setAccumulatedDistanceMm(kCellDistanceThresholdMm, 100);
    tracker.update();

    const GridPosition firstRead = tracker.currentState().position;
    for (int i = 0; i < 5; ++i) {
        const GridPosition repeatedRead = tracker.currentState().position;
        TEST_ASSERT_EQUAL_INT16(firstRead.row, repeatedRead.row);
        TEST_ASSERT_EQUAL_INT16(firstRead.col, repeatedRead.col);
    }

    // Uma nova atualização continua funcionando normalmente após as consultas.
    odometry.setAccumulatedDistanceMm(kCellDistanceThresholdMm, 100 + kMinCellAdvanceIntervalMs + 1);
    tracker.update();
    TEST_ASSERT_EQUAL_INT16(-2, tracker.currentState().position.row);
}

// Integração: uma sequência longa e variada de avanços/rotações nunca
// deixa a posição/orientação em um estado indefinido ou fora do esperado.
static void test_position_tracker_never_loses_reference_across_mixed_sequence(void) {
    MockOdometryReader odometry;
    MockOrientationSensor orientationSensor;
    PositionTracker tracker(odometry, orientationSensor);

    uint32_t clockMs = 0;
    int16_t yawDegrees = 0;

    auto advanceOneCell = [&]() {
        clockMs += kMinCellAdvanceIntervalMs + 1;
        odometry.setAccumulatedDistanceMm(kCellDistanceThresholdMm, clockMs);
        tracker.update();
    };
    auto rotate = [&](int16_t deltaDegrees) {
        clockMs += 10;
        yawDegrees = static_cast<int16_t>(yawDegrees + deltaDegrees);
        orientationSensor.setYawDegrees(yawDegrees, clockMs);
        tracker.update();
    };

    // avança, gira 90, avança, gira 180, avança, avança, gira -90, avança
    advanceOneCell();
    rotate(90);
    advanceOneCell();
    rotate(180);
    advanceOneCell();
    advanceOneCell();
    rotate(-90);
    advanceOneCell();

    // Sequência aplicada a partir de (0,0)/Norte:
    // avança Norte -> (-1,0); gira 90 -> Leste; avança Leste -> (-1,1);
    // gira 180 -> Oeste; avança Oeste -> (-1,0); avança Oeste -> (-1,-1);
    // gira -90 -> Sul; avança Sul -> (0,-1).
    TEST_ASSERT_EQUAL(Orientation::South, tracker.currentState().orientation);
    TEST_ASSERT_EQUAL_INT16(0, tracker.currentState().position.row);
    TEST_ASSERT_EQUAL_INT16(-1, tracker.currentState().position.col);
}

// Microbenchmark de custo de computação de update(), isolado e intercalado
// com consultas repetidas a currentState(). Mede apenas o processamento
// puro, sem I/O real de sensor nem agendamento de tarefas do sistema.
static void test_update_latency_isolated_vs_interleaved_with_queries(void) {
    constexpr int kIterations = 1000;
    constexpr uint32_t kMaxAverageMicros = 500;
    constexpr double kMaxInterleavedOverheadRatio = 0.10;

    MockOdometryReader odometryIsolated;
    MockOrientationSensor orientationIsolated;
    PositionTracker isolatedTracker(odometryIsolated, orientationIsolated);
    odometryIsolated.setAccumulatedDistanceMm(10, 0);

    const auto isolatedStart = std::chrono::steady_clock::now();
    for (int i = 0; i < kIterations; ++i) {
        isolatedTracker.update();
    }
    const auto isolatedEnd = std::chrono::steady_clock::now();
    const double isolatedAverageMicros =
        std::chrono::duration<double, std::micro>(isolatedEnd - isolatedStart).count() / kIterations;

    MockOdometryReader odometryInterleaved;
    MockOrientationSensor orientationInterleaved;
    PositionTracker interleavedTracker(odometryInterleaved, orientationInterleaved);
    odometryInterleaved.setAccumulatedDistanceMm(10, 0);

    const auto interleavedStart = std::chrono::steady_clock::now();
    for (int i = 0; i < kIterations; ++i) {
        interleavedTracker.update();
        // Simula consultas repetidas de consumidores (mapeamento/telemetria)
        // entre cada ciclo de atualização.
        volatile auto consumed = interleavedTracker.currentState();
        (void)consumed;
    }
    const auto interleavedEnd = std::chrono::steady_clock::now();
    const double interleavedAverageMicros =
        std::chrono::duration<double, std::micro>(interleavedEnd - interleavedStart).count() /
        kIterations;

    TEST_ASSERT_LESS_OR_EQUAL_UINT32(kMaxAverageMicros, static_cast<uint32_t>(isolatedAverageMicros));
    TEST_ASSERT_LESS_OR_EQUAL_UINT32(kMaxAverageMicros,
                                      static_cast<uint32_t>(interleavedAverageMicros));

    const double overheadRatio =
        (interleavedAverageMicros - isolatedAverageMicros) / isolatedAverageMicros;
    TEST_ASSERT_TRUE(overheadRatio < kMaxInterleavedOverheadRatio);
}

int main(int argc, char** argv) {
    UNITY_BEGIN();

    RUN_TEST(test_mock_odometry_and_orientation_return_programmed_values);
    RUN_TEST(test_rotate_orientation_clockwise_90_cycles_through_all_four);
    RUN_TEST(test_rotate_orientation_counter_clockwise_90_reverses_the_cycle);
    RUN_TEST(test_rotate_orientation_180_is_the_opposite_direction);
    RUN_TEST(test_apply_cell_advance_moves_one_step_per_orientation);
    RUN_TEST(test_try_confirm_rotation_90_degrees_within_tolerance);
    RUN_TEST(test_try_confirm_rotation_180_degrees_within_tolerance);
    RUN_TEST(test_try_confirm_rotation_below_threshold_does_not_confirm);
    RUN_TEST(test_position_tracker_confirms_advance_and_updates_position);
    RUN_TEST(test_position_tracker_below_threshold_does_not_change_position);
    RUN_TEST(test_position_tracker_deduplicates_rapid_advance_confirmations);
    RUN_TEST(test_position_tracker_advance_behavior_independent_of_maze_size);
    RUN_TEST(test_position_tracker_rotation_updates_orientation_then_next_advance);
    RUN_TEST(test_position_tracker_current_state_available_before_first_update);
    RUN_TEST(test_position_tracker_repeated_queries_do_not_affect_state);
    RUN_TEST(test_position_tracker_never_loses_reference_across_mixed_sequence);
    RUN_TEST(test_update_latency_isolated_vs_interleaved_with_queries);

    return UNITY_END();
}
