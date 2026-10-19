// Teste nativo do adaptador PositionTracker -> MazeMap.
// pio test -e native -f test_tracking_adapter
#include <unity.h>

#include "navigation/MazeMap.h"
#include "navigation/TrackingAdapter.h"

using namespace mm;
using tracking::Orientation;
using tracking::TrackingState;

namespace {
MazeMap map;

TrackingState st(int16_t row, int16_t col, Orientation o)
{
    TrackingState s;
    s.position.row = row;
    s.position.col = col;
    s.orientation  = o;
    return s;
}
}  // namespace

void setUp(void) { map.init(8, 4); }
void tearDown(void) {}

void test_orientation_para_dir(void)
{
    TEST_ASSERT_EQUAL(Dir::North, toDir(Orientation::North));
    TEST_ASSERT_EQUAL(Dir::East,  toDir(Orientation::East));
    TEST_ASSERT_EQUAL(Dir::South, toDir(Orientation::South));
    TEST_ASSERT_EQUAL(Dir::West,  toDir(Orientation::West));
}

void test_x_e_col_y_e_row(void)
{
    // row=2, col=5 -> célula (x=5, y=2)
    TEST_ASSERT_TRUE(updateMapFromTracking(map, st(2, 5, Orientation::East), false, false, false));
    TEST_ASSERT_EQUAL_UINT8(1, map.get(5, 2)->visited);
    TEST_ASSERT_EQUAL_UINT8(0, map.get(2, 2)->visited);
}

void test_parede_a_frente_olhando_norte_fica_em_y_menos_1(void)
{
    // robô em row=2,col=1 olhando Norte, parede à frente
    updateMapFromTracking(map, st(2, 1, Orientation::North), true, false, false);
    TEST_ASSERT_TRUE(map.get(1, 2)->hasWall(Dir::North));
    TEST_ASSERT_TRUE(map.get(1, 1)->hasWall(Dir::South));   // célula row-1 (mesma parede)
}

void test_esquerda_de_quem_olha_leste_e_norte(void)
{
    updateMapFromTracking(map, st(1, 1, Orientation::East), false, true, false);
    TEST_ASSERT_TRUE(map.get(1, 1)->hasWall(Dir::North));
    TEST_ASSERT_FALSE(map.get(1, 1)->hasWall(Dir::South));
}

void test_fora_do_labirinto(void)
{
    TEST_ASSERT_FALSE(updateMapFromTracking(map, st(4, 0, Orientation::North), false, false, false));
    TEST_ASSERT_FALSE(updateMapFromTracking(map, st(-1, 0, Orientation::North), false, false, false));
}

int main(int, char **)
{
    UNITY_BEGIN();
    RUN_TEST(test_orientation_para_dir);
    RUN_TEST(test_x_e_col_y_e_row);
    RUN_TEST(test_parede_a_frente_olhando_norte_fica_em_y_menos_1);
    RUN_TEST(test_esquerda_de_quem_olha_leste_e_norte);
    RUN_TEST(test_fora_do_labirinto);
    return UNITY_END();
}
