#include <unity.h>

#include <chrono>

#include "mock_wall_sensor.h"
#include "navigation/WallPerception.h"

using perception::Direction;
using perception::RawEcho;
using perception::WallCellState;
using perception::WallPerception;
using perception::kCorridorWallThresholdMm;
using perception::test::MockWallSensor;

void setUp(void) {}
void tearDown(void) {}

// T009 — teste de sanidade: confirma que o toolchain host + MockWallSensor +
// IWallSensor funcionam, antes de qualquer lógica de percepção existir.
static void test_mock_wall_sensor_returns_enqueued_reading(void) {
    MockWallSensor sensor;
    sensor.enqueue(Direction::Front, RawEcho{120, true, 1000});

    const RawEcho echo = sensor.read(Direction::Front);

    TEST_ASSERT_EQUAL_UINT16(120, echo.rawDistanceMm);
    TEST_ASSERT_TRUE(echo.hasEcho);
    TEST_ASSERT_EQUAL_UINT32(1000, echo.timestampMs);
}

// T012 — máquina de estados de SensorFaultState (FR-010). Escrito antes de
// T013/T014 existirem (estado "red" esperado em TDD).
static void test_fault_state_set_after_five_consecutive_invalid_readings(void) {
    MockWallSensor sensor;
    WallPerception perception(sensor);

    sensor.enqueue(Direction::Front, RawEcho{100, true, 1});
    sensor.enqueue(Direction::Left, RawEcho{100, true, 1});
    sensor.enqueue(Direction::Right, RawEcho{100, true, 1});
    perception.update();
    TEST_ASSERT_FALSE(perception.hasFault(Direction::Front));

    // 4 leituras inválidas (fisicamente impossíveis, 0mm) seguidas: ainda sem falha.
    for (int i = 0; i < 4; ++i) {
        const uint32_t timestamp = static_cast<uint32_t>(i + 2);
        sensor.enqueue(Direction::Front, RawEcho{0, true, timestamp});
        sensor.enqueue(Direction::Left, RawEcho{100, true, timestamp});
        sensor.enqueue(Direction::Right, RawEcho{100, true, timestamp});
        perception.update();
    }
    TEST_ASSERT_FALSE(perception.hasFault(Direction::Front));

    // 5ª leitura inválida consecutiva: falha deve ser sinalizada.
    sensor.enqueue(Direction::Front, RawEcho{0, true, 6});
    sensor.enqueue(Direction::Left, RawEcho{100, true, 6});
    sensor.enqueue(Direction::Right, RawEcho{100, true, 6});
    perception.update();
    TEST_ASSERT_TRUE(perception.hasFault(Direction::Front));

    // Outras direções nunca afetadas durante o episódio.
    TEST_ASSERT_FALSE(perception.hasFault(Direction::Left));
    TEST_ASSERT_FALSE(perception.hasFault(Direction::Right));

    // 1 leitura válida: falha removida automaticamente.
    sensor.enqueue(Direction::Front, RawEcho{100, true, 7});
    sensor.enqueue(Direction::Left, RawEcho{100, true, 7});
    sensor.enqueue(Direction::Right, RawEcho{100, true, 7});
    perception.update();
    TEST_ASSERT_FALSE(perception.hasFault(Direction::Front));
}

// T015 — detecção binária de parede (US1, SC-001, Acceptance Scenarios 1-3).
static void test_wall_detection_binary_flags_per_direction(void) {
    MockWallSensor sensor;
    WallPerception perception(sensor);

    sensor.enqueue(Direction::Front, RawEcho{static_cast<uint16_t>(kCorridorWallThresholdMm - 10), true, 10});
    sensor.enqueue(Direction::Left, RawEcho{static_cast<uint16_t>(kCorridorWallThresholdMm + 50), true, 10});
    sensor.enqueue(Direction::Right, RawEcho{kCorridorWallThresholdMm, true, 10});
    perception.update();

    const WallCellState& state = perception.currentState();
    TEST_ASSERT_TRUE(state.wallFront);
    TEST_ASSERT_FALSE(state.wallLeft);
    TEST_ASSERT_TRUE(state.wallRight);  // caso de fronteira: distância == limiar conta como parede
}

static void test_wall_detection_boundary_is_deterministic(void) {
    MockWallSensor sensor;
    WallPerception perception(sensor);

    for (int i = 0; i < 5; ++i) {
        const uint32_t timestamp = static_cast<uint32_t>(i + 1);
        sensor.enqueue(Direction::Front, RawEcho{kCorridorWallThresholdMm, true, timestamp});
        sensor.enqueue(Direction::Left, RawEcho{kCorridorWallThresholdMm, true, timestamp});
        sensor.enqueue(Direction::Right, RawEcho{kCorridorWallThresholdMm, true, timestamp});
        perception.update();
        TEST_ASSERT_TRUE(perception.currentState().wallFront);
    }
}

// FR-011: a lógica de detecção não recebe nem depende do tamanho do
// labirinto — não há nenhum parâmetro de tamanho de labirinto em
// WallPerception; este teste confirma que o comportamento de detecção é
// sempre o mesmo, independente de qual "labirinto" está sendo percorrido.
static void test_wall_detection_independent_of_maze_size(void) {
    MockWallSensor sensor;
    WallPerception perception(sensor);

    sensor.enqueue(Direction::Front, RawEcho{30, true, 0});
    sensor.enqueue(Direction::Left, RawEcho{30, true, 0});
    sensor.enqueue(Direction::Right, RawEcho{30, true, 0});
    perception.update();

    const WallCellState& state = perception.currentState();
    TEST_ASSERT_TRUE(state.wallFront);
    TEST_ASSERT_TRUE(state.wallLeft);
    TEST_ASSERT_TRUE(state.wallRight);
}

// T019 — medição numérica lateral suavizada (US2, SC-002/SC-004).
static void test_lateral_distance_reflects_filtered_value_not_raw_spike(void) {
    MockWallSensor sensor;
    WallPerception perception(sensor);

    const uint16_t stable = 100;
    const uint16_t spike = 400;

    sensor.enqueue(Direction::Left, RawEcho{stable, true, 0});
    sensor.enqueue(Direction::Front, RawEcho{300, true, 0});
    sensor.enqueue(Direction::Right, RawEcho{300, true, 0});
    perception.update();

    sensor.enqueue(Direction::Left, RawEcho{stable, true, 10});
    sensor.enqueue(Direction::Front, RawEcho{300, true, 10});
    sensor.enqueue(Direction::Right, RawEcho{300, true, 10});
    perception.update();

    sensor.enqueue(Direction::Left, RawEcho{spike, true, 20});  // pico isolado de ruído
    sensor.enqueue(Direction::Front, RawEcho{300, true, 20});
    sensor.enqueue(Direction::Right, RawEcho{300, true, 20});
    perception.update();

    const uint16_t afterSpike = perception.lateralDistanceMm(Direction::Left);
    TEST_ASSERT_LESS_THAN_UINT16(spike, afterSpike);  // suavizado, não reflete o pico bruto
}

static void test_same_snapshot_is_not_filtered_twice(void) {
    MockWallSensor sensor;
    WallPerception perception(sensor);

    sensor.enqueue(Direction::Front, RawEcho{300, true, 42});
    sensor.enqueue(Direction::Left, RawEcho{120, true, 42});
    sensor.enqueue(Direction::Right, RawEcho{300, true, 42});
    perception.update();
    TEST_ASSERT_EQUAL_UINT16(120, perception.lateralDistanceMm(Direction::Left));

    // Uma leitura reprocessada sem novo timestamp deve ser ignorada.
    sensor.enqueue(Direction::Front, RawEcho{300, true, 42});
    sensor.enqueue(Direction::Left, RawEcho{400, true, 42});
    sensor.enqueue(Direction::Right, RawEcho{300, true, 42});
    perception.update();
    TEST_ASSERT_EQUAL_UINT16(120, perception.lateralDistanceMm(Direction::Left));
}

static void test_lateral_distance_updates_every_cycle(void) {
    MockWallSensor sensor;
    WallPerception perception(sensor);

    sensor.enqueue(Direction::Left, RawEcho{50, true, 0});
    sensor.enqueue(Direction::Front, RawEcho{300, true, 0});
    sensor.enqueue(Direction::Right, RawEcho{300, true, 0});
    perception.update();
    const uint16_t first = perception.lateralDistanceMm(Direction::Left);

    sensor.enqueue(Direction::Left, RawEcho{200, true, 10});
    sensor.enqueue(Direction::Front, RawEcho{300, true, 10});
    sensor.enqueue(Direction::Right, RawEcho{300, true, 10});
    perception.update();
    const uint16_t second = perception.lateralDistanceMm(Direction::Left);

    TEST_ASSERT_NOT_EQUAL(first, second);
}

// T021 — integridade do WallCellState publicado para telemetria (US3, SC-005).
static void test_current_state_has_no_missing_fields_after_update(void) {
    MockWallSensor sensor;
    WallPerception perception(sensor);

    sensor.enqueue(Direction::Front, RawEcho{50, true, 123});
    sensor.enqueue(Direction::Left, RawEcho{60, true, 123});
    sensor.enqueue(Direction::Right, RawEcho{70, true, 123});
    perception.update();

    const WallCellState& state = perception.currentState();
    TEST_ASSERT_EQUAL_UINT32(123, state.timestampMs);
    TEST_ASSERT_TRUE(state.frontValid);
    TEST_ASSERT_TRUE(state.leftValid);
    TEST_ASSERT_TRUE(state.rightValid);
    TEST_ASSERT_EQUAL_UINT16(50, state.distFrontMm);
    TEST_ASSERT_EQUAL_UINT16(60, state.distLeftMm);
    TEST_ASSERT_EQUAL_UINT16(70, state.distRightMm);
}

// Uma amostra fisicamente inválida permanece desconhecida para os módulos
// consumidores; não deve ser interpretada como parede nem como corredor livre.
static void test_invalid_current_reading_is_marked_unknown(void) {
    MockWallSensor sensor;
    WallPerception perception(sensor);

    sensor.enqueue(Direction::Front, RawEcho{0, true, 77});  // inválida: abaixo do alcance mínimo
    sensor.enqueue(Direction::Left, RawEcho{100, true, 77});
    sensor.enqueue(Direction::Right, RawEcho{0, false, 77}); // sem eco = caminho livre
    perception.update();

    const WallCellState& state = perception.currentState();
    TEST_ASSERT_FALSE(state.frontValid);
    TEST_ASSERT_FALSE(state.wallFront);
    TEST_ASSERT_EQUAL_UINT16(0, state.distFrontMm);

    TEST_ASSERT_TRUE(state.leftValid);
    TEST_ASSERT_FALSE(state.wallLeft);
    TEST_ASSERT_TRUE(state.rightValid);
    TEST_ASSERT_FALSE(state.wallRight);
}

// T023 — microbenchmark do custo de computação de update() (FR-008/SC-002).
// Mede apenas o processamento puro, isolado do agendamento real do
// FreeRTOS/I2C/UART (ver tasks.md T029 para a dependência cruzada de
// SC-003 com HU-04/HU-05).
static void test_update_computation_latency_budget(void) {
    MockWallSensor sensor;
    WallPerception perception(sensor);

    constexpr int kIterations = 1000;
    for (int i = 0; i < kIterations; ++i) {
        const uint32_t timestamp = static_cast<uint32_t>(i + 1);
        sensor.enqueue(Direction::Front, RawEcho{100, true, timestamp});
        sensor.enqueue(Direction::Left, RawEcho{100, true, timestamp});
        sensor.enqueue(Direction::Right, RawEcho{100, true, timestamp});
    }

    const auto start = std::chrono::steady_clock::now();
    for (int i = 0; i < kIterations; ++i) {
        perception.update();
    }
    const auto end = std::chrono::steady_clock::now();

    const uint32_t averageNanos = static_cast<uint32_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count() / kIterations);

    // Orçamento total do ciclo é 10ms (10.000.000ns, FR-008/SC-002); exigimos
    // aqui uma folga generosa (<1ms de computação pura) para deixar margem
    // ao custo real de I/O do sensor quando o hardware existir.
    TEST_ASSERT_LESS_THAN_UINT32(1000000u, averageNanos);
}

int main(int argc, char** argv) {
    UNITY_BEGIN();

    RUN_TEST(test_mock_wall_sensor_returns_enqueued_reading);
    RUN_TEST(test_fault_state_set_after_five_consecutive_invalid_readings);
    RUN_TEST(test_wall_detection_binary_flags_per_direction);
    RUN_TEST(test_wall_detection_boundary_is_deterministic);
    RUN_TEST(test_wall_detection_independent_of_maze_size);
    RUN_TEST(test_lateral_distance_reflects_filtered_value_not_raw_spike);
    RUN_TEST(test_same_snapshot_is_not_filtered_twice);
    RUN_TEST(test_lateral_distance_updates_every_cycle);
    RUN_TEST(test_current_state_has_no_missing_fields_after_update);
    RUN_TEST(test_invalid_current_reading_is_marked_unknown);
    RUN_TEST(test_update_computation_latency_budget);

    return UNITY_END();
}
