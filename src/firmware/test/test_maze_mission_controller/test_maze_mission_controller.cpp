// Integração nativa HU-00/01/02/03/05/06: pio test -e native -f test_maze_mission_controller
#include <unity.h>
#include <cstring>
#include <cstdio>

#include "config/maze_config.hpp"
#include "navigation/MazeMissionController.h"
#include "navigation/PositionTracker.h"
#include "navigation/RobotStateEstimator.h"
#include "../test_position_tracker/mock_odometry_reader.h"
#include "../test_position_tracker/mock_orientation_sensor.h"
#include "../test_wall_perception/mock_wall_sensor.h"

namespace {

class FakeStorage final : public mm::IMapStorage {
public:
    uint8_t data[mm::MazeMap::kBlobMaxSize]{};
    size_t length = 0;
    bool failSave = false;

    bool save(const uint8_t* input, size_t size) override {
        if (failSave || size > sizeof(data)) return false;
        std::memcpy(data, input, size);
        length = size;
        return true;
    }
    size_t load(uint8_t* output, size_t capacity) override {
        if (length == 0 || length > capacity) return 0;
        std::memcpy(output, data, length);
        return length;
    }
};

class MissionFixture {
public:
    FakeStorage storage;
    tracking::test::MockOdometryReader odometry;
    tracking::test::MockOrientationSensor orientation;
    perception::test::MockWallSensor wallSensor;
    tracking::PositionTracker tracker;
    perception::WallPerception wallPerception;
    state_estimation::RobotStateEstimator estimator;
    mm::MazeMap map;
    mm::MazeMap truth;
    nav::MazeMissionController mission;
    uint32_t timestamp = 1000;
    int16_t yaw = 0;

    MissionFixture()
        : tracker(odometry, orientation),
          wallPerception(wallSensor),
          estimator(tracker, wallPerception),
          mission(map, storage, estimator) {}

    bool begin(const MazeConfig& config, bool addObstacle = false) {
        if (!truth.init(config.cols, config.rows)) return false;
        for (uint8_t x = 0; x < config.cols; ++x) {
            truth.setWall(x, 0, mm::Dir::North, true);
            truth.setWall(x, config.rows - 1, mm::Dir::South, true);
        }
        for (uint8_t y = 0; y < config.rows; ++y) {
            truth.setWall(0, y, mm::Dir::West, true);
            truth.setWall(config.cols - 1, y, mm::Dir::East, true);
        }
        if (addObstacle && config.cols > 2 && config.rows > 1) {
            // Obriga o Flood Fill a recalcular quando a parede for percebida.
            truth.setWall(1, 0, mm::Dir::East, true);
        }
        yaw = 0;
        orientation.setYawDegrees(yaw, timestamp);
        odometry.setAccumulatedDistanceMm(0, timestamp);
        return mission.begin(config, 0);
    }

    void enqueueCellSnapshot() {
        const mm::Cell* cell = truth.get(mission.x(), mission.y());
        const mm::Dir heading = mission.heading();
        if (cell == nullptr) return;

        const mm::Dir front = heading;
        const mm::Dir left = mm::MazeMap::relToAbs(heading, mm::Rel::Left);
        const mm::Dir right = mm::MazeMap::relToAbs(heading, mm::Rel::Right);
        for (int sample = 0; sample < 3; ++sample) {
            ++timestamp;
            wallSensor.enqueue(perception::Direction::Front,
                cell->hasWall(front)
                    ? perception::RawEcho{60, true, timestamp}
                    : perception::RawEcho{0, false, timestamp});
            wallSensor.enqueue(perception::Direction::Left,
                cell->hasWall(left)
                    ? perception::RawEcho{60, true, timestamp}
                    : perception::RawEcho{0, false, timestamp});
            wallSensor.enqueue(perception::Direction::Right,
                cell->hasWall(right)
                    ? perception::RawEcho{60, true, timestamp}
                    : perception::RawEcho{0, false, timestamp});
        }
        for (int sample = 0; sample < 3; ++sample) estimator.updateWalls();
    }

    nav::MissionUpdate observe() {
        enqueueCellSnapshot();
        return mission.observeCurrentCell();
    }

    nav::MissionUpdate completeMotion(nav::MissionUpdate action) {
        if (action.state == nav::MissionState::Failed ||
            action.action == nav::MissionAction::Brake) return action;

        if (action.action == nav::MissionAction::TurnLeft90 ||
            action.action == nav::MissionAction::TurnRight90 ||
            action.action == nav::MissionAction::TurnAround180) {
            int16_t delta = 0;
            if (action.action == nav::MissionAction::TurnLeft90) delta = -90;
            else if (action.action == nav::MissionAction::TurnRight90) delta = 90;
            else delta = -180;
            int value = static_cast<int>(yaw) + delta;
            while (value > 180) value -= 360;
            while (value < -180) value += 360;
            yaw = static_cast<int16_t>(value);
            timestamp += 250;
            orientation.setYawDegrees(yaw, timestamp);
            odometry.setAccumulatedDistanceMm(0, timestamp);
            estimator.updatePose();
            return mission.onMotionFinished(true);
        }

        const bool moveForward = action.action == nav::MissionAction::MoveForwardOneCell;
        const bool moveBackward = action.action == nav::MissionAction::MoveBackwardOneCell;
        if (!moveForward && !moveBackward) return action;
        const auto direction = moveForward ? mission.heading() :
            mm::MazeMap::relToAbs(mission.heading(),mm::Rel::Back);
        TEST_ASSERT_FALSE(truth.get(mission.x(),mission.y())->hasWall(direction));
        timestamp += 300;
        const int16_t signedDistance = moveForward
            ? tracking::kCellDistanceThresholdMm
            : static_cast<int16_t>(-tracking::kCellDistanceThresholdMm);
        odometry.setAccumulatedDistanceMm(signedDistance, timestamp);
        estimator.updatePose();
        return mission.onMotionFinished(true);
    }
};

bool reachesGoal(MazeCorner corner, Direction initialHeading) {
    MissionFixture fixture;
    const MazeConfig config = makeMazeConfig(MazeFormat::Maze4x4, corner, initialHeading);
    if (!fixture.begin(config)) return false;

    for (uint16_t step = 0; step < 100; ++step) {
        if (fixture.mission.state() != nav::MissionState::AwaitingObservation) return false;
        nav::MissionUpdate action = fixture.observe();
        if (action.state == nav::MissionState::Completed) {
            mm::MazeMap restored;
            return fixture.mission.x() == config.goal.col &&
                   fixture.mission.y() == config.goal.row &&
                   fixture.storage.length > 0 && restored.load(fixture.storage) &&
                   restored.get(config.goal.col, config.goal.row)->visited;
        }
        if (action.state == nav::MissionState::Failed) return false;
        action = fixture.completeMotion(action);
        if (action.state == nav::MissionState::Failed) return false;
    }
    return false;
}

bool reachesGoalWithDetour() {
    MissionFixture fixture;
    const MazeConfig config = makeMazeConfig(
        MazeFormat::Maze8x4, MazeCorner::TopLeft, Direction::East);
    if (!fixture.begin(config, true)) return false;

    for (uint16_t step = 0; step < 150; ++step) {
        if (fixture.mission.state() != nav::MissionState::AwaitingObservation) return false;
        nav::MissionUpdate action = fixture.observe();
        if (action.state == nav::MissionState::Completed) {
            mm::MazeMap restored;
            return fixture.mission.x() == config.goal.col &&
                   fixture.mission.y() == config.goal.row &&
                   restored.load(fixture.storage) &&
                   restored.get(config.goal.col, config.goal.row)->visited;
        }
        if (action.state == nav::MissionState::Failed) return false;
        action = fixture.completeMotion(action);
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
    const MazeConfig maze12x4 = makeMazeConfig(
        MazeFormat::Maze12x4, MazeCorner::BottomRight, Direction::North);
    TEST_ASSERT_TRUE(isValidMazeConfig(maze12x4));
    TEST_ASSERT_EQUAL_UINT8(12, maze12x4.cols);
    TEST_ASSERT_EQUAL_UINT8(4, maze12x4.rows);
    TEST_ASSERT_EQUAL_UINT8(11, maze12x4.start.col);
    TEST_ASSERT_EQUAL_UINT8(3, maze12x4.start.row);
    const MazeConfig maze8x4 = makeMazeConfig(
        MazeFormat::Maze8x4, MazeCorner::TopLeft, Direction::East);
    TEST_ASSERT_EQUAL_UINT8(8, maze8x4.cols);
    TEST_ASSERT_EQUAL_UINT8(4, maze8x4.rows);
}

void test_rejeita_objetivo_nao_oposto_ao_inicio(void) {
    MazeConfig config = makeMazeConfig(
        MazeFormat::Maze8x4, MazeCorner::TopRight, Direction::West);
    config.goal = {0, 3};
    TEST_ASSERT_FALSE(isValidMazeConfig(config));
}

void test_missao_usa_pose_estimada_e_persiste_mapa_final(void) {
    TEST_ASSERT_TRUE(reachesGoal(MazeCorner::TopLeft, Direction::East));
    TEST_ASSERT_TRUE(reachesGoal(MazeCorner::TopRight, Direction::West));
    TEST_ASSERT_TRUE(reachesGoal(MazeCorner::BottomLeft, Direction::North));
    TEST_ASSERT_TRUE(reachesGoal(MazeCorner::BottomRight, Direction::North));
}

void test_missao_recalcula_rota_ao_identificar_obstaculo_8x4(void) {
    TEST_ASSERT_TRUE(reachesGoalWithDetour());
}


void test_erro_antes_de_iniciar_e_configuracao_invalida(void) {
    MissionFixture fixture;
    const nav::MissionUpdate beforeStart = fixture.mission.observeCurrentCell();
    TEST_ASSERT_EQUAL(nav::MissionState::Failed, beforeStart.state);
    TEST_ASSERT_EQUAL(nav::NavigationStatus::InvalidObservation, beforeStart.navigationStatus);

    MazeConfig invalid = makeMazeConfig(
        MazeFormat::Maze4x4, MazeCorner::TopLeft, Direction::East);
    invalid.goal = {0, 0};
    TEST_ASSERT_FALSE(fixture.mission.begin(invalid, 0));
    TEST_ASSERT_EQUAL(nav::MissionState::NotStarted, fixture.mission.state());
}

void test_snapshot_nao_inicializado_e_falha_de_persistencia(void) {
    const MazeConfig config = makeMazeConfig(
        MazeFormat::Maze4x4, MazeCorner::TopLeft, Direction::East);

    MissionFixture noSnapshot;
    TEST_ASSERT_TRUE(noSnapshot.begin(config));
    const nav::MissionUpdate noState = noSnapshot.mission.observeCurrentCell();
    TEST_ASSERT_EQUAL(nav::MissionState::Failed, noState.state);
    TEST_ASSERT_EQUAL(nav::NavigationStatus::InvalidObservation, noState.navigationStatus);

    MissionFixture persistenceFailure;
    TEST_ASSERT_TRUE(persistenceFailure.begin(config));
    persistenceFailure.storage.failSave = true;
    const nav::MissionUpdate failedSave = persistenceFailure.observe();
    TEST_ASSERT_EQUAL(nav::MissionState::Failed, failedSave.state);
    TEST_ASSERT_EQUAL(nav::NavigationStatus::PersistenceError, failedSave.navigationStatus);
}

void test_retaguarda_desconhecida_exige_giro_e_nova_observacao(void) {
    MissionFixture fixture;
    const auto config=makeMazeConfig(MazeFormat::Maze4x4,MazeCorner::TopLeft,Direction::North);
    TEST_ASSERT_TRUE(fixture.begin(config));
    fixture.truth.setWall(0,0,mm::Dir::East,true);
    const auto scan=fixture.observe();
    TEST_ASSERT_EQUAL(nav::MissionState::Turning,scan.state);
    TEST_ASSERT_EQUAL(nav::MissionAction::TurnRight90,scan.action);
    const auto stopped=fixture.completeMotion(scan);
    TEST_ASSERT_EQUAL(nav::MissionState::AwaitingObservation,stopped.state);
    TEST_ASSERT_EQUAL(nav::MissionAction::None,stopped.action);
    TEST_ASSERT_EQUAL(0,fixture.mission.x());TEST_ASSERT_EQUAL(0,fixture.mission.y());
    const auto next=fixture.observe();
    TEST_ASSERT_EQUAL(nav::MissionState::Turning,next.state);
    TEST_ASSERT_EQUAL(nav::MissionAction::TurnRight90,next.action);
    TEST_ASSERT_EQUAL(nav::MissionState::AwaitingObservation,fixture.completeMotion(next).state);
    TEST_ASSERT_EQUAL(0,fixture.mission.x());TEST_ASSERT_EQUAL(0,fixture.mission.y());
    TEST_ASSERT_EQUAL(nav::MissionState::AwaitingObservation,fixture.completeMotion(fixture.observe()).state);
    TEST_ASSERT_EQUAL(0,fixture.mission.x());TEST_ASSERT_EQUAL(1,fixture.mission.y());
}
void test_sensor_frontal_sem_eco_invalido_nao_autoriza_movimento(void) {
    MissionFixture fixture;
    const auto config=makeMazeConfig(MazeFormat::Maze4x4,MazeCorner::TopLeft,Direction::East);
    TEST_ASSERT_TRUE(fixture.begin(config));
    for(int i=0;i<3;++i){++fixture.timestamp;
        fixture.wallSensor.enqueue(perception::Direction::Front,{0,false,fixture.timestamp,false});
        fixture.wallSensor.enqueue(perception::Direction::Left,{60,true,fixture.timestamp});
        fixture.wallSensor.enqueue(perception::Direction::Right,{60,true,fixture.timestamp});
        fixture.estimator.updateWalls();
    }
    const auto update=fixture.mission.observeCurrentCell();
    TEST_ASSERT_EQUAL(nav::MissionState::Failed,update.state);
    TEST_ASSERT_EQUAL(nav::MissionAction::Brake,update.action);
    TEST_ASSERT_EQUAL(0,fixture.mission.x());TEST_ASSERT_EQUAL(0,fixture.mission.y());
}

void test_finalizacao_de_movimento_rejeita_resultados_invalidos(void) {
    const MazeConfig config = makeMazeConfig(
        MazeFormat::Maze4x4, MazeCorner::TopLeft, Direction::East);

    MissionFixture unsuccessful;
    TEST_ASSERT_TRUE(unsuccessful.begin(config));
    const nav::MissionUpdate stopped = unsuccessful.mission.onMotionFinished(false);
    TEST_ASSERT_EQUAL(nav::MissionState::Failed, stopped.state);

    MissionFixture wrongState;
    TEST_ASSERT_TRUE(wrongState.begin(config));
    const nav::MissionUpdate notMoving = wrongState.mission.onMotionFinished(true);
    TEST_ASSERT_EQUAL(nav::MissionState::Failed, notMoving.state);

    MissionFixture wrongHeading;
    const MazeConfig facingNorth = makeMazeConfig(
        MazeFormat::Maze4x4, MazeCorner::TopLeft, Direction::North);
    TEST_ASSERT_TRUE(wrongHeading.begin(facingNorth));
    const nav::MissionUpdate turn = wrongHeading.observe();
    TEST_ASSERT_EQUAL(nav::MissionState::Turning, turn.state);
    const nav::MissionUpdate failedTurn = wrongHeading.mission.onMotionFinished(true);
    TEST_ASSERT_EQUAL(nav::MissionState::Failed, failedTurn.state);
}

void test_pose_fora_dos_limites_e_rejeitada_e_conversoes_usam_255(void) {
    const MazeConfig config = makeMazeConfig(
        MazeFormat::Maze4x4, MazeCorner::TopLeft, Direction::East);

    const tracking::TrackingState badPoses[] = {
        {{-1, 0}, tracking::Orientation::East, 1001},
        {{0, -1}, tracking::Orientation::East, 1002},
        {{4, 0}, tracking::Orientation::East, 1003},
        {{0, 4}, tracking::Orientation::East, 1004},
    };

    for (size_t i = 0; i < sizeof(badPoses) / sizeof(badPoses[0]); ++i) {
        MissionFixture fixture;
        TEST_ASSERT_TRUE(fixture.begin(config));
        fixture.estimator.initialize(badPoses[i], 0);
        fixture.estimator.updateWalls();

        if (badPoses[i].position.col < 0) {
            TEST_ASSERT_EQUAL_UINT8(255, fixture.mission.x());
        }
        if (badPoses[i].position.row < 0) {
            TEST_ASSERT_EQUAL_UINT8(255, fixture.mission.y());
        }
        const nav::MissionUpdate update = fixture.mission.observeCurrentCell();
        TEST_ASSERT_EQUAL(nav::MissionState::Failed, update.state);
        TEST_ASSERT_EQUAL(nav::NavigationStatus::InvalidObservation, update.navigationStatus);
    }
}

void test_sessao_invalida_salta_para_invalid_observation_do_navegador(void) {
    MissionFixture fixture;
    const MazeConfig config = makeMazeConfig(
        MazeFormat::Maze4x4, MazeCorner::TopLeft, Direction::East);
    TEST_ASSERT_TRUE(fixture.begin(config));

    nav::MissionUpdate action = fixture.observe();
    TEST_ASSERT_EQUAL(nav::MissionState::Moving, action.state);
    action = fixture.completeMotion(action);
    TEST_ASSERT_EQUAL(nav::MissionState::AwaitingObservation, action.state);

    // O estimador é substituído por uma pose válida, porém incompatível com a
    // última célula vista pelo AutonomousNavigator (salto de mais de uma célula).
    const tracking::TrackingState jumpedPose{{2, 2}, tracking::Orientation::East, 2000};
    fixture.estimator.initialize(jumpedPose, 0);
    fixture.estimator.updateWalls();
    const nav::MissionUpdate jumped = fixture.mission.observeCurrentCell();
    TEST_ASSERT_EQUAL(nav::MissionState::Failed, jumped.state);
    TEST_ASSERT_EQUAL(nav::NavigationStatus::InvalidObservation, jumped.navigationStatus);
}


void test_gira_a_esquerda_e_recua_quando_o_caminho_exige(void) {
    MissionFixture leftTurn;
    const MazeConfig startSouth = makeMazeConfig(
        MazeFormat::Maze4x4, MazeCorner::TopLeft, Direction::South);
    TEST_ASSERT_TRUE(leftTurn.begin(startSouth));
    // Fecha a frente; o corredor para a meta fica à esquerda do heading inicial.
    leftTurn.truth.setWall(startSouth.start.col, startSouth.start.row, mm::Dir::South, true);
    const nav::MissionUpdate turn = leftTurn.observe();
    TEST_ASSERT_EQUAL(nav::MissionState::Turning, turn.state);
    TEST_ASSERT_EQUAL(nav::MissionAction::TurnLeft90, turn.action);
    const auto afterTurn=leftTurn.completeMotion(turn);
    TEST_ASSERT_EQUAL(nav::MissionState::AwaitingObservation,afterTurn.state);
    TEST_ASSERT_EQUAL(nav::MissionAction::None,afterTurn.action);
    TEST_ASSERT_EQUAL(0,leftTurn.mission.x());TEST_ASSERT_EQUAL(0,leftTurn.mission.y());
    TEST_ASSERT_EQUAL(nav::MissionAction::MoveForwardOneCell,leftTurn.observe().action);

    MissionFixture reverse;
    const MazeConfig startEast = makeMazeConfig(
        MazeFormat::Maze4x4, MazeCorner::TopLeft, Direction::East);
    TEST_ASSERT_TRUE(reverse.begin(startEast));
    nav::MissionUpdate first = reverse.observe();
    TEST_ASSERT_EQUAL(nav::MissionAction::MoveForwardOneCell, first.action);
    first = reverse.completeMotion(first);
    TEST_ASSERT_EQUAL(nav::MissionState::AwaitingObservation, first.state);

    // Cria um beco à frente do robô na célula (1,0); voltar à célula anterior
    // é a única rota conhecida, portanto o controlador deve ordenar movimento reverso.
    reverse.truth.setWall(1, 0, mm::Dir::East, true);
    reverse.truth.setWall(1, 0, mm::Dir::South, true);
    const nav::MissionUpdate back = reverse.observe();
    TEST_ASSERT_EQUAL(nav::MissionState::Moving, back.state);
    TEST_ASSERT_EQUAL(nav::MissionAction::MoveBackwardOneCell, back.action);
}


void test_finalizacao_detecta_pose_nao_confirmada_e_deriva(void) {
    const MazeConfig config = makeMazeConfig(
        MazeFormat::Maze4x4, MazeCorner::TopLeft, Direction::East);

    MissionFixture noAdvance;
    TEST_ASSERT_TRUE(noAdvance.begin(config));
    const nav::MissionUpdate move = noAdvance.observe();
    TEST_ASSERT_EQUAL(nav::MissionState::Moving, move.state);
    // O hardware informa conclusão, mas o estimador não confirmou a célula destino.
    const nav::MissionUpdate stalled = noAdvance.mission.onMotionFinished(true);
    TEST_ASSERT_EQUAL(nav::MissionState::Failed, stalled.state);

    MissionFixture wrongRow;
    TEST_ASSERT_TRUE(wrongRow.begin(config));
    const nav::MissionUpdate secondMove = wrongRow.observe();
    TEST_ASSERT_EQUAL(nav::MissionState::Moving, secondMove.state);
    const tracking::TrackingState rowDrift{{1, 1}, tracking::Orientation::East, 2000};
    wrongRow.estimator.initialize(rowDrift, 0);
    const nav::MissionUpdate drifted = wrongRow.mission.onMotionFinished(true);
    TEST_ASSERT_EQUAL(nav::MissionState::Failed, drifted.state);

    MissionFixture outOfBounds;
    TEST_ASSERT_TRUE(outOfBounds.begin(config));
    const nav::MissionUpdate thirdMove = outOfBounds.observe();
    TEST_ASSERT_EQUAL(nav::MissionState::Moving, thirdMove.state);
    const tracking::TrackingState invalidPose{{-1, 1}, tracking::Orientation::East, 2001};
    outOfBounds.estimator.initialize(invalidPose, 0);
    const nav::MissionUpdate rejected = outOfBounds.mission.onMotionFinished(true);
    TEST_ASSERT_EQUAL(nav::MissionState::Failed, rejected.state);
}

void test_fail_publico_interrompe_missao(void) {
    MissionFixture fixture;
    const MazeConfig config = makeMazeConfig(
        MazeFormat::Maze4x4, MazeCorner::TopLeft, Direction::East);
    TEST_ASSERT_TRUE(fixture.begin(config));
    const nav::MissionUpdate failed = fixture.mission.fail();
    TEST_ASSERT_EQUAL(nav::MissionState::Failed, failed.state);
    TEST_ASSERT_EQUAL(nav::MissionAction::Brake, failed.action);
    TEST_ASSERT_EQUAL(nav::NavigationStatus::InvalidObservation, failed.navigationStatus);
}


uint32_t mazeRandom(uint32_t& seed) { seed=1664525u*seed+1013904223u;return seed; }
void carveMaze(mm::MazeMap& map,uint32_t seed,bool loops) {
 const int width=map.width(),height=map.height(),cells=width*height;
 for(int y=0;y<height;++y)for(int x=0;x<width;++x)
  for(int d=0;d<4;++d)map.setWall(x,y,static_cast<mm::Dir>(d),true);
 bool seen[256]{};int stack[256]{},depth=1;seen[0]=true;
 const int dx[]={0,1,0,-1},dy[]={-1,0,1,0};
 while(depth){
  const int at=stack[depth-1],x=at%width,y=at/width;
  int choices[4],count=0;
  for(int d=0;d<4;++d){const int nx=x+dx[d],ny=y+dy[d];
   if(nx>=0&&ny>=0&&nx<width&&ny<height&&!seen[ny*width+nx])choices[count++]=d;}
  if(!count){--depth;continue;}
  const int d=choices[mazeRandom(seed)%count];
  const int next=(y+dy[d])*width+x+dx[d];
  map.setWall(x,y,static_cast<mm::Dir>(d),false);seen[next]=true;stack[depth++]=next;
 }
 for(int i=0;i<cells;++i)TEST_ASSERT_TRUE(seen[i]);
 if(loops)for(int y=0;y<height;++y)for(int x=0;x<width;++x){
  if(x+1<width && mazeRandom(seed)%4==0)map.setWall(x,y,mm::Dir::East,false);
  if(y+1<height && mazeRandom(seed)%4==0)map.setWall(x,y,mm::Dir::South,false);
 }
}
void test_complete_mission_in_768_generated_mazes() {
 unsigned completed=0,reverses=0;
 const MazeFormat formats[]={MazeFormat::Maze4x4,MazeFormat::Maze8x4,MazeFormat::Maze12x4};
 for(auto format:formats)for(unsigned corner=0;corner<4;++corner)for(unsigned axis=0;axis<2;++axis)
  for(uint32_t seed=1;seed<=32;++seed){
   const Direction inward=axis==0 ? ((corner&1)?Direction::West:Direction::East)
                                 : ((corner&2)?Direction::North:Direction::South);
   const auto config=makeMazeConfig(format,static_cast<MazeCorner>(corner),inward);
   MissionFixture fixture;TEST_ASSERT_TRUE(fixture.begin(config));
   carveMaze(fixture.truth,seed,(seed&1)!=0);
   bool done=false;
   for(unsigned step=0;step<4096;++step){
    auto action=fixture.observe();
    if(action.state==nav::MissionState::Completed){done=true;break;}
    char context[180];std::snprintf(context,sizeof(context),"width=%u corner=%u axis=%u seed=%lu step=%u x=%u y=%u heading=%u nav=%u",config.cols,corner,axis,static_cast<unsigned long>(seed),step,fixture.mission.x(),fixture.mission.y(),static_cast<unsigned>(fixture.mission.heading()),static_cast<unsigned>(action.navigationStatus));
    TEST_ASSERT_NOT_EQUAL_MESSAGE(nav::MissionState::Failed,action.state,context);
    mm::Rel relative=mm::Rel::Front;
    switch(action.action){
     case nav::MissionAction::TurnLeft90:relative=mm::Rel::Left;break;
     case nav::MissionAction::TurnRight90:relative=mm::Rel::Right;break;
     case nav::MissionAction::MoveBackwardOneCell:++reverses;relative=mm::Rel::Back;break;
     case nav::MissionAction::TurnAround180:relative=mm::Rel::Back;break;
     default:TEST_ASSERT_EQUAL(nav::MissionAction::MoveForwardOneCell,action.action);break;
    }
    const auto direction=mm::MazeMap::relToAbs(fixture.mission.heading(),relative);
    if(action.action==nav::MissionAction::MoveForwardOneCell || action.action==nav::MissionAction::MoveBackwardOneCell)
     TEST_ASSERT_FALSE(fixture.truth.get(fixture.mission.x(),fixture.mission.y())->hasWall(direction));
    action=fixture.completeMotion(action);
    TEST_ASSERT_EQUAL(nav::MissionState::AwaitingObservation,action.state);
   }
   TEST_ASSERT_TRUE_MESSAGE(done,"Mission did not converge within simulation bound");
   TEST_ASSERT_EQUAL(config.goal.col,fixture.mission.x());TEST_ASSERT_EQUAL(config.goal.row,fixture.mission.y());
   TEST_ASSERT_TRUE(fixture.storage.length>0);++completed;
  }
 TEST_ASSERT_EQUAL(768,completed);TEST_ASSERT_TRUE(reverses>0);
}
void test_disconnected_goal_ends_without_crossing_wall() {
 MissionFixture fixture;auto config=makeMazeConfig(MazeFormat::Maze4x4,MazeCorner::TopLeft,Direction::East);
 TEST_ASSERT_TRUE(fixture.begin(config));
 for(int d=0;d<4;++d)fixture.truth.setWall(3,3,static_cast<mm::Dir>(d),true);
 bool stopped=false;
 for(int step=0;step<256;++step){auto action=fixture.observe();
  if(action.state==nav::MissionState::Failed){TEST_ASSERT_EQUAL(nav::NavigationStatus::NoPath,action.navigationStatus);stopped=true;break;}
  TEST_ASSERT_NOT_EQUAL(nav::MissionState::Completed,action.state);
  TEST_ASSERT_EQUAL(nav::MissionState::AwaitingObservation,fixture.completeMotion(action).state);
 }
 TEST_ASSERT_TRUE(stopped);
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_complete_mission_in_768_generated_mazes);
    RUN_TEST(test_disconnected_goal_ends_without_crossing_wall);
    RUN_TEST(test_configura_origem_objetivo_nos_quatro_cantos);
    RUN_TEST(test_rejeita_objetivo_nao_oposto_ao_inicio);
    RUN_TEST(test_missao_usa_pose_estimada_e_persiste_mapa_final);
    RUN_TEST(test_missao_recalcula_rota_ao_identificar_obstaculo_8x4);
    RUN_TEST(test_erro_antes_de_iniciar_e_configuracao_invalida);
    RUN_TEST(test_snapshot_nao_inicializado_e_falha_de_persistencia);
    RUN_TEST(test_retaguarda_desconhecida_exige_giro_e_nova_observacao);
    RUN_TEST(test_sensor_frontal_sem_eco_invalido_nao_autoriza_movimento);
    RUN_TEST(test_finalizacao_de_movimento_rejeita_resultados_invalidos);
    RUN_TEST(test_pose_fora_dos_limites_e_rejeitada_e_conversoes_usam_255);
    RUN_TEST(test_sessao_invalida_salta_para_invalid_observation_do_navegador);
    RUN_TEST(test_gira_a_esquerda_e_recua_quando_o_caminho_exige);
    RUN_TEST(test_finalizacao_detecta_pose_nao_confirmada_e_deriva);
    RUN_TEST(test_fail_publico_interrompe_missao);
    return UNITY_END();
}
