// Teste de integração das HU-00 e HU-03. Executar com:
// pio test -e native -f test_autonomous_navigator
#include <unity.h>

#include <cstring>

#include "hal/IMapStorage.h"
#include "navigation/AutonomousNavigator.h"

using mm::Dir;
using nav::AutonomousNavigator;
using nav::NavigationStatus;
using nav::WallReadings;

namespace {

class FakeStorage : public mm::IMapStorage {
public:
    uint8_t bytes[mm::MazeMap::kBlobMaxSize]{};
    size_t length = 0;
    bool failSave = false;

    bool save(const uint8_t *data, size_t len) override {
        if (failSave || len > sizeof(bytes)) return false;
        std::memcpy(bytes, data, len);
        length = len;
        return true;
    }

    size_t load(uint8_t *out, size_t maxLen) override {
        if (length == 0 || length > maxLen) return 0;
        std::memcpy(out, bytes, length);
        return length;
    }
};

bool truthWall(const mm::MazeMap &truth, int x, int y, Dir d)
{
    const mm::Cell *cell = truth.get(x, y);
    return cell == nullptr || cell->hasWall(d);
}

void closeBorders(mm::MazeMap &truth, uint8_t width, uint8_t height)
{
    for (uint8_t x = 0; x < width; ++x) {
        truth.setWall(x, 0, Dir::North, true);
        truth.setWall(x, height - 1, Dir::South, true);
    }
    for (uint8_t y = 0; y < height; ++y) {
        truth.setWall(0, y, Dir::West, true);
        truth.setWall(width - 1, y, Dir::East, true);
    }
}

WallReadings readTruth(const mm::MazeMap &truth, int x, int y, Dir heading)
{
    WallReadings readings = {
        truthWall(truth, x, y, mm::MazeMap::relToAbs(heading, mm::Rel::Front)),
        truthWall(truth, x, y, mm::MazeMap::relToAbs(heading, mm::Rel::Left)),
        truthWall(truth, x, y, mm::MazeMap::relToAbs(heading, mm::Rel::Right))
    };
    return readings;
}

void addDetours(mm::MazeMap &truth)
{
    // Mesmo conjunto de obstáculos usado para validar a exploração 8x4.
    truth.setWall(0, 0, Dir::East, true);
    truth.setWall(0, 1, Dir::East, true);
    truth.setWall(0, 2, Dir::East, true);
    truth.setWall(2, 3, Dir::North, true);
    truth.setWall(3, 3, Dir::East, true);
    truth.setWall(3, 1, Dir::East, true);
    truth.setWall(3, 0, Dir::East, true);
}

bool reachesGoal(uint8_t width, uint8_t height, bool withDetours)
{
    mm::MazeMap map;
    mm::MazeMap truth;
    FakeStorage storage;
    if (!map.init(width, height) || !truth.init(width, height)) return false;
    closeBorders(truth, width, height);
    if (withDetours) addDetours(truth);

    AutonomousNavigator navigator(map, storage);
    if (!navigator.begin()) return false;

    int x = 0;
    int y = 0;
    Dir heading = Dir::East;
    for (int step = 0; step < 300; ++step) {
        const nav::NavigationResult result =
            navigator.observeCell(x, y, heading, readTruth(truth, x, y, heading));
        if (result.status == NavigationStatus::AtGoal) {
            mm::MazeMap restored;
            return x == width - 1 && y == height - 1 &&
                   restored.load(storage) && restored.visitedCount() == map.visitedCount();
        }
        if (result.status != NavigationStatus::Move ||
            truthWall(truth, x, y, result.nextDirection)) {
            return false;
        }

        heading = result.nextDirection;
        switch (heading) {
        case Dir::North: --y; break;
        case Dir::East:  ++x; break;
        case Dir::South: ++y; break;
        case Dir::West:  --x; break;
        }
    }
    return false;
}

}  // namespace

void test_begin_exige_mapa_inicializado(void)
{
    mm::MazeMap map;
    FakeStorage storage;
    AutonomousNavigator navigator(map, storage);
    TEST_ASSERT_FALSE(navigator.begin());
    TEST_ASSERT_EQUAL(NavigationStatus::NotInitialized,
        navigator.observeCell(0, 0, Dir::East, {false, false, false}).status);
}

void test_observacao_atualiza_mapa_e_persiste(void)
{
    mm::MazeMap map;
    FakeStorage storage;
    TEST_ASSERT_TRUE(map.init(4, 4));
    AutonomousNavigator navigator(map, storage);
    TEST_ASSERT_TRUE(navigator.begin());

    const nav::NavigationResult result = navigator.observeCell(
        0, 0, Dir::East, {true, true, false});
    TEST_ASSERT_EQUAL(NavigationStatus::Move, result.status);
    TEST_ASSERT_TRUE(map.get(0, 0)->visited);
    TEST_ASSERT_TRUE(map.get(0, 0)->hasWall(Dir::East));
    TEST_ASSERT_TRUE(map.get(0, 0)->hasWall(Dir::North));
    TEST_ASSERT_TRUE(storage.length > 0);

    mm::MazeMap restored;
    TEST_ASSERT_TRUE(restored.load(storage));
    TEST_ASSERT_EQUAL_UINT16(1, restored.visitedCount());
    TEST_ASSERT_EQUAL_UINT8(map.get(0, 0)->walls, restored.get(0, 0)->walls);
}


void test_movimento_confirmado_marca_corredor_como_livre(void)
{
    mm::MazeMap map;
    FakeStorage storage;
    TEST_ASSERT_TRUE(map.init(4, 4));
    AutonomousNavigator navigator(map, storage);
    TEST_ASSERT_TRUE(navigator.begin());

    // Primeira célula observada; segunda observação confirma um avanço ao Sul.
    navigator.observeCell(0, 0, Dir::East, {false, false, false});
    navigator.observeCell(0, 1, Dir::South, {false, false, false});

    TEST_ASSERT_TRUE(map.get(0, 0)->isKnown(Dir::South));
    TEST_ASSERT_FALSE(map.get(0, 0)->hasWall(Dir::South));
    TEST_ASSERT_TRUE(map.get(0, 1)->isKnown(Dir::North));
    TEST_ASSERT_FALSE(map.get(0, 1)->hasWall(Dir::North));
}

void test_nao_emite_movimento_se_persistencia_falha(void)
{
    mm::MazeMap map;
    FakeStorage storage;
    TEST_ASSERT_TRUE(map.init(4, 4));
    AutonomousNavigator navigator(map, storage);
    TEST_ASSERT_TRUE(navigator.begin());
    storage.failSave = true;

    TEST_ASSERT_EQUAL(NavigationStatus::PersistenceError,
        navigator.observeCell(0, 0, Dir::East, {false, false, false}).status);
}

void test_resolve_labirintos_oficiais(void)
{
    TEST_ASSERT_TRUE(reachesGoal(4, 4, false));
    TEST_ASSERT_TRUE(reachesGoal(8, 4, true));
    TEST_ASSERT_TRUE(reachesGoal(12, 4, false));
}


void test_begin_rejeita_origem_e_objetivo_fora_do_mapa(void)
{
    mm::MazeMap map;
    FakeStorage storage;
    TEST_ASSERT_TRUE(map.init(4, 4));
    AutonomousNavigator navigator(map, storage);

    TEST_ASSERT_FALSE(navigator.begin(4, 0, 3, 3)); // origem inválida
    TEST_ASSERT_FALSE(navigator.begin(0, 0, 4, 3)); // objetivo inválido
    TEST_ASSERT_FALSE(navigator.initialized());
    TEST_ASSERT_TRUE(navigator.begin(0, 0, 3, 3));
}

void test_observacoes_repetidas_e_movimento_nas_quatro_direcoes(void)
{
    mm::MazeMap map;
    FakeStorage storage;
    TEST_ASSERT_TRUE(map.init(4, 4));
    AutonomousNavigator navigator(map, storage);
    TEST_ASSERT_TRUE(navigator.begin(0, 0, 3, 3));

    // Primeira observação (sem posição anterior) e repetição na mesma célula.
    navigator.observeCell(1, 1, Dir::East, {false, false, false});
    navigator.observeCell(1, 1, Dir::East, {false, false, false});

    // Alternar vizinhos cobre os ramos East, West, South e North do delta.
    navigator.observeCell(2, 1, Dir::East, {false, false, false});
    navigator.observeCell(1, 1, Dir::West, {false, false, false});
    navigator.observeCell(1, 2, Dir::South, {false, false, false});
    navigator.observeCell(1, 1, Dir::North, {false, false, false});

    TEST_ASSERT_TRUE(map.get(1, 1)->visited);
    TEST_ASSERT_TRUE(map.get(1, 1)->isKnown(Dir::East));
    TEST_ASSERT_TRUE(map.get(1, 1)->isKnown(Dir::West));
    TEST_ASSERT_TRUE(map.get(1, 1)->isKnown(Dir::North));
    TEST_ASSERT_TRUE(map.get(1, 1)->isKnown(Dir::South));
}

void test_observacao_saliente_e_salto_nao_adjacente_sao_rejeitados(void)
{
    mm::MazeMap map;
    FakeStorage storage;
    TEST_ASSERT_TRUE(map.init(4, 4));
    AutonomousNavigator navigator(map, storage);
    TEST_ASSERT_TRUE(navigator.begin(0, 0, 3, 3));

    TEST_ASSERT_EQUAL(NavigationStatus::InvalidObservation,
        navigator.observeCell(-1, 0, Dir::East, {false, false, false}).status);
    TEST_ASSERT_EQUAL(NavigationStatus::Move,
        navigator.observeCell(0, 0, Dir::East, {false, false, false}).status);
    TEST_ASSERT_EQUAL(NavigationStatus::InvalidObservation,
        navigator.observeCell(2, 0, Dir::East, {false, false, false}).status);
}

void test_decide_sem_caminho(void)
{
    mm::MazeMap map;
    FakeStorage storage;
    TEST_ASSERT_TRUE(map.init(4, 4));
    AutonomousNavigator navigator(map, storage);
    TEST_ASSERT_TRUE(navigator.begin(0, 0, 3, 3));

    // No canto (0,0), fecha as duas saídas internas; as outras duas são bordas.
    const nav::NavigationResult result = navigator.observeCell(
        0, 0, Dir::East, {true, true, true});
    TEST_ASSERT_EQUAL(NavigationStatus::NoPath, result.status);
}


void test_deltas_diagonais_exercitam_falhas_das_condicoes_de_adjacencia(void)
{
    const int deltas[][2] = {
        {1, 1},   // dx == 1, mas dy != 0
        {-1, 1},  // dx == -1, mas dy != 0
        {0, 2},   // dx == 0, mas dy não é +1 nem -1
        {2, 0},   // dx não corresponde a nenhum vizinho
    };

    for (const auto& delta : deltas) {
        mm::MazeMap map;
        FakeStorage storage;
        TEST_ASSERT_TRUE(map.init(4, 4));
        AutonomousNavigator navigator(map, storage);
        TEST_ASSERT_TRUE(navigator.begin(0, 0, 3, 3));
        TEST_ASSERT_EQUAL(NavigationStatus::Move,
            navigator.observeCell(1, 1, Dir::East, {false, false, false}).status);
        TEST_ASSERT_EQUAL(NavigationStatus::InvalidObservation,
            navigator.observeCell(1 + delta[0], 1 + delta[1], Dir::East,
                                  {false, false, false}).status);
    }
}

int main(int, char **)
{
    UNITY_BEGIN();
    RUN_TEST(test_begin_exige_mapa_inicializado);
    RUN_TEST(test_observacao_atualiza_mapa_e_persiste);
    RUN_TEST(test_movimento_confirmado_marca_corredor_como_livre);
    RUN_TEST(test_nao_emite_movimento_se_persistencia_falha);
    RUN_TEST(test_resolve_labirintos_oficiais);
    RUN_TEST(test_begin_rejeita_origem_e_objetivo_fora_do_mapa);
    RUN_TEST(test_observacoes_repetidas_e_movimento_nas_quatro_direcoes);
    RUN_TEST(test_observacao_saliente_e_salto_nao_adjacente_sao_rejeitados);
    RUN_TEST(test_decide_sem_caminho);
    RUN_TEST(test_deltas_diagonais_exercitam_falhas_das_condicoes_de_adjacencia);
    return UNITY_END();
}
