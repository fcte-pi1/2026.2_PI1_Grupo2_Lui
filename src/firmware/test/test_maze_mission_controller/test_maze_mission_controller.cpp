// Integração nativa HU-00/HU-03/HU-05/HU-06: pio test -e native -f test_maze_mission_controller
#include <unity.h>
#include <cstring>

#include "config/maze_config.hpp"
#include "navigation/MazeMissionController.h"

namespace {

class FakeStorage final : public mm::IMapStorage {
public:
    uint8_t data[mm::MazeMap::kBlobMaxSize]{};
    size_t length = 0;
    bool save(const uint8_t *input, size_t size) override {
        if (size > sizeof(data)) return false;
        std::memcpy(data, input, size);
        length = size;
        return true;
    }
    size_t load(uint8_t *output, size_t capacity) override {
        if (length == 0 || length > capacity) return 0;
        std::memcpy(output, data, length);
        return length;
    }
};

bool reachesConfiguredGoal(MazeCorner corner, Direction initialHeading) {
    mm::MazeMap map;
    FakeStorage storage;
    nav::MazeMissionController mission(map, storage);
    const MazeConfig config = makeMazeConfig(MazeFormat::Maze4x4, corner, initialHeading);
    if (!mission.begin(config)) return false;

    for (uint16_t step = 0; step < 80; ++step) {
        if (mission.state() == nav::MissionState::AwaitingObservation) {
            nav::MissionUpdate next = mission.observeCell({false, false, false});
            if (next.state == nav::MissionState::Completed) {
                mm::MazeMap restored;
                return mission.x() == config.goal.col &&
                       mission.y() == config.goal.row &&
                       storage.length > 0 && restored.load(storage) &&
                       restored.get(config.goal.col, config.goal.row)->visited;
            }
            if (next.state == nav::MissionState::Failed ||
                next.action == nav::MissionAction::Brake) return false;
            if (next.action == nav::MissionAction::TurnLeft90 ||
                next.action == nav::MissionAction::TurnRight90 ||
                next.action == nav::MissionAction::TurnAround180) {
                next = mission.onMotionFinished(true); // rotação concluída
                if (next.action != nav::MissionAction::MoveForwardOneCell) return false;
            } else if (next.action != nav::MissionAction::MoveForwardOneCell) {
                return false;
            }
            next = mission.onMotionFinished(true); // avanço de uma célula concluído
            if (next.state == nav::MissionState::Failed) return false;
        } else {
            return false;
        }
    }
    return false;
}


bool reachesEightByFourMazeWithDetour() {
    constexpr uint8_t width = 4;
    constexpr uint8_t height = 8;
    mm::MazeMap map;
    mm::MazeMap truth;
    FakeStorage storage;
    if (!truth.init(width, height)) return false;
    for (uint8_t x = 0; x < width; ++x) {
        truth.setWall(x, 0, mm::Dir::North, true);
        truth.setWall(x, height - 1, mm::Dir::South, true);
    }
    for (uint8_t y = 0; y < height; ++y) {
        truth.setWall(0, y, mm::Dir::West, true);
        truth.setWall(width - 1, y, mm::Dir::East, true);
    }
    // Obstáculo no primeiro corredor que exige recalcular a rota ao ser visto.
    truth.setWall(1, 0, mm::Dir::East, true);

    nav::MazeMissionController mission(map, storage);
    const MazeConfig config = makeMazeConfig(
        MazeFormat::Maze8x4, MazeCorner::TopLeft, Direction::East);
    if (!mission.begin(config)) return false;

    for (uint16_t step = 0; step < 100; ++step) {
        if (mission.state() != nav::MissionState::AwaitingObservation) return false;
        const uint8_t x = mission.x();
        const uint8_t y = mission.y();
        const mm::Dir heading = mission.heading();
        const mm::Cell *cell = truth.get(x, y);
        if (cell == nullptr) return false;
        const nav::WallReadings walls = {
            cell->hasWall(mm::MazeMap::relToAbs(heading, mm::Rel::Front)),
            cell->hasWall(mm::MazeMap::relToAbs(heading, mm::Rel::Left)),
            cell->hasWall(mm::MazeMap::relToAbs(heading, mm::Rel::Right))
        };
        nav::MissionUpdate action = mission.observeCell(walls);
        if (action.state == nav::MissionState::Completed) {
            mm::MazeMap restored;
            return mission.x() == config.goal.col && mission.y() == config.goal.row &&
                   restored.load(storage) &&
                   restored.get(config.goal.col, config.goal.row)->visited;
        }
        if (action.state == nav::MissionState::Failed ||
            action.action == nav::MissionAction::Brake) return false;
        if (action.action == nav::MissionAction::TurnLeft90 ||
            action.action == nav::MissionAction::TurnRight90 ||
            action.action == nav::MissionAction::TurnAround180) {
            action = mission.onMotionFinished(true);
            if (action.action != nav::MissionAction::MoveForwardOneCell) return false;
        } else if (action.action != nav::MissionAction::MoveForwardOneCell) {
            return false;
        }
        const mm::Dir moveDirection = mission.heading();
        if (cell->hasWall(moveDirection)) return false; // nunca atravessa parede conhecida pela verdade
        action = mission.onMotionFinished(true);
        if (action.state == nav::MissionState::Failed) return false;
    }
    return false;
}

}  // namespace

void test_configura_origem_objetivo_nos_quatro_cantos(void) {
    TEST_ASSERT_TRUE(isValidMazeConfig(makeMazeConfig(
        MazeFormat::Maze4x4, MazeCorner::TopLeft, Direction::East)));
    TEST_ASSERT_TRUE(isValidMazeConfig(makeMazeConfig(
        MazeFormat::Maze4x4, MazeCorner::TopRight, Direction::West)));
    TEST_ASSERT_TRUE(isValidMazeConfig(makeMazeConfig(
        MazeFormat::Maze8x4, MazeCorner::BottomLeft, Direction::North)));
    TEST_ASSERT_TRUE(isValidMazeConfig(makeMazeConfig(
        MazeFormat::Maze12x4, MazeCorner::BottomRight, Direction::North)));
}

void test_rejeita_objetivo_nao_oposto_ao_inicio(void) {
    MazeConfig config = makeMazeConfig(
        MazeFormat::Maze8x4, MazeCorner::TopRight, Direction::West);
    config.goal = {0, 3}; // TopRight deveria apontar para BottomLeft (linha 7, coluna 0).
    TEST_ASSERT_FALSE(isValidMazeConfig(config));
}

void test_missao_usa_objetivo_configurado_e_persiste_final(void) {
    TEST_ASSERT_TRUE(reachesConfiguredGoal(MazeCorner::TopLeft, Direction::East));
    TEST_ASSERT_TRUE(reachesConfiguredGoal(MazeCorner::TopRight, Direction::West));
    TEST_ASSERT_TRUE(reachesConfiguredGoal(MazeCorner::BottomLeft, Direction::North));
    TEST_ASSERT_TRUE(reachesConfiguredGoal(MazeCorner::BottomRight, Direction::North));
}

void test_missao_recalcula_rota_ao_identificar_obstaculo_8x4(void) {
    TEST_ASSERT_TRUE(reachesEightByFourMazeWithDetour());
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_configura_origem_objetivo_nos_quatro_cantos);
    RUN_TEST(test_rejeita_objetivo_nao_oposto_ao_inicio);
    RUN_TEST(test_missao_usa_objetivo_configurado_e_persiste_final);
    RUN_TEST(test_missao_recalcula_rota_ao_identificar_obstaculo_8x4);
    return UNITY_END();
}
