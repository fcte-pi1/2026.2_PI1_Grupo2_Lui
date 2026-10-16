// Teste nativo da HU-03 (Unity).  pio test -e native -f test_maze_solver
#include <unity.h>

#include "navigation/MazeMap.h"
#include "navigation/MazeSolver.h"

using namespace mm;
using nav::Decision;
using nav::MazeSolver;

namespace {

MazeMap map;        // fora da stack
MazeMap truth;      // labirinto "real" usado na simulação de exploração

/* Há parede entre (x,y) e o vizinho em d? Borda do labirinto conta como parede. */
bool truthWall(int x, int y, Dir d)
{
    const Cell *c = truth.get(x, y);
    return c == nullptr || c->hasWall(d);
}

/* Fecha as bordas externas do labirinto `truth` (w x h). */
void closeBorders(uint8_t w, uint8_t h)
{
    for (uint8_t x = 0; x < w; x++) {
        truth.setWall(x, 0, Dir::North, true);
        truth.setWall(x, h - 1, Dir::South, true);
    }
    for (uint8_t y = 0; y < h; y++) {
        truth.setWall(0, y, Dir::West, true);
        truth.setWall(w - 1, y, Dir::East, true);
    }
}

}  // namespace

void setUp(void) { map.init(4, 4); }
void tearDown(void) {}

void test_set_goal_valida_limites(void)
{
    MazeSolver s(map);
    TEST_ASSERT_FALSE(s.setGoal(4, 0));
    TEST_ASSERT_FALSE(s.setGoal(0, 4));
    TEST_ASSERT_TRUE(s.setGoal(3, 3));
    TEST_ASSERT_TRUE(s.isGoal(3, 3));
    TEST_ASSERT_FALSE(s.isGoal(0, 0));
}

void test_recompute_sem_objetivo_falha(void)
{
    MazeSolver s(map);
    TEST_ASSERT_FALSE(s.recompute());
    Dir out = Dir::North;
    TEST_ASSERT_EQUAL(Decision::NoPath, s.decide(0, 0, Dir::East, out));
}

void test_distancia_labirinto_aberto(void)
{
    MazeSolver s(map);
    s.setGoal(3, 3);
    TEST_ASSERT_TRUE(s.recompute());
    TEST_ASSERT_EQUAL_UINT16(0, s.distance(3, 3));
    TEST_ASSERT_EQUAL_UINT16(6, s.distance(0, 0));
    TEST_ASSERT_EQUAL_UINT16(MazeSolver::kUnreachable, s.distance(9, 9));
}

void test_distancia_12x4(void)
{
    static MazeMap big;
    big.init(12, 4);
    MazeSolver s(big);
    s.setGoal(11, 3);
    s.recompute();
    TEST_ASSERT_EQUAL_UINT16(14, s.distance(0, 0));
}

void test_empate_prefere_seguir_em_frente(void)
{
    MazeSolver s(map);
    s.setGoal(3, 3);
    Dir out = Dir::North;
    TEST_ASSERT_EQUAL(Decision::Move, s.decide(0, 0, Dir::East, out));
    TEST_ASSERT_EQUAL(Dir::East, out);
    TEST_ASSERT_EQUAL(Decision::Move, s.decide(0, 0, Dir::South, out));
    TEST_ASSERT_EQUAL(Dir::South, out);
}

void test_parede_conhecida_forca_desvio(void)
{
    map.setWall(0, 0, Dir::East, true);
    MazeSolver s(map);
    s.setGoal(3, 0);
    Dir out = Dir::North;
    TEST_ASSERT_EQUAL(Decision::Move, s.decide(0, 0, Dir::East, out));
    TEST_ASSERT_EQUAL(Dir::South, out);
    TEST_ASSERT_EQUAL_UINT16(5, s.distance(0, 0));   // S, L, L, L, N
}

void test_parede_do_lado_vizinho_tambem_bloqueia(void)
{
    // Só o lado oeste da célula (1,0) é conhecido; a outra face ainda não.
    map.setWall(1, 0, Dir::West, true);
    MazeSolver s(map);
    s.setGoal(3, 0);
    s.recompute();
    TEST_ASSERT_TRUE(s.distance(0, 0) > 3);
}

void test_lado_desconhecido_conta_como_livre(void)
{
    MazeSolver s(map);       // mapa sem nenhuma parede medida
    s.setGoal(3, 0);
    Dir out = Dir::South;
    TEST_ASSERT_EQUAL(Decision::Move, s.decide(0, 0, Dir::East, out));
    TEST_ASSERT_EQUAL(Dir::East, out);
}

void test_chegou_no_objetivo(void)
{
    MazeSolver s(map);
    s.setGoal(3, 3);
    Dir out = Dir::North;
    TEST_ASSERT_EQUAL(Decision::AtGoal, s.decide(3, 3, Dir::East, out));
}

void test_sem_caminho(void)
{
    map.setWall(3, 3, Dir::North, true);
    map.setWall(3, 3, Dir::West, true);
    MazeSolver s(map);
    s.setGoal(3, 3);
    Dir out = Dir::North;
    TEST_ASSERT_EQUAL(Decision::NoPath, s.decide(0, 0, Dir::East, out));
    TEST_ASSERT_EQUAL_UINT16(MazeSolver::kUnreachable, s.distance(0, 0));
}

void test_posicao_fora_do_mapa(void)
{
    MazeSolver s(map);
    s.setGoal(3, 3);
    Dir out = Dir::North;
    TEST_ASSERT_EQUAL(Decision::NoPath, s.decide(7, 7, Dir::East, out));
}

/* Simula o robô explorando um labirinto 8x4 com becos e uma única passagem
 * entre as duas metades. Só enxerga frente/esquerda/direita, como no robô. */
void test_exploracao_converge_ate_o_objetivo(void)
{
    map.init(8, 4);
    truth.init(8, 4);
    closeBorders(8, 4);
    truth.setWall(0, 0, Dir::East, true);
    truth.setWall(0, 1, Dir::East, true);
    truth.setWall(0, 2, Dir::East, true);
    truth.setWall(2, 3, Dir::North, true);
    truth.setWall(3, 3, Dir::East, true);
    truth.setWall(3, 1, Dir::East, true);
    truth.setWall(3, 0, Dir::East, true);

    MazeSolver s(map);
    TEST_ASSERT_TRUE(s.setGoal(7, 3));

    int x = 0, y = 0;
    Dir heading = Dir::East;
    int steps = 0;
    bool reached = false;

    while (steps < 300) {
        map.updateCell(x, y, heading,
                       truthWall(x, y, MazeMap::relToAbs(heading, Rel::Front)),
                       truthWall(x, y, MazeMap::relToAbs(heading, Rel::Left)),
                       truthWall(x, y, MazeMap::relToAbs(heading, Rel::Right)));
        Dir next = Dir::North;
        const Decision dec = s.decide(x, y, heading, next);
        if (dec == Decision::AtGoal) {
            reached = true;
            break;
        }
        TEST_ASSERT_EQUAL(Decision::Move, dec);
        TEST_ASSERT_FALSE(truthWall(x, y, next));   // nunca atravessa parede real
        heading = next;
        x += (next == Dir::East) - (next == Dir::West);
        y += (next == Dir::South) - (next == Dir::North);
        steps++;
    }
    TEST_ASSERT_TRUE(reached);
    TEST_ASSERT_TRUE(steps < 100);
}

int main(int, char **)
{
    UNITY_BEGIN();
    RUN_TEST(test_set_goal_valida_limites);
    RUN_TEST(test_recompute_sem_objetivo_falha);
    RUN_TEST(test_distancia_labirinto_aberto);
    RUN_TEST(test_distancia_12x4);
    RUN_TEST(test_empate_prefere_seguir_em_frente);
    RUN_TEST(test_parede_conhecida_forca_desvio);
    RUN_TEST(test_parede_do_lado_vizinho_tambem_bloqueia);
    RUN_TEST(test_lado_desconhecido_conta_como_livre);
    RUN_TEST(test_chegou_no_objetivo);
    RUN_TEST(test_sem_caminho);
    RUN_TEST(test_posicao_fora_do_mapa);
    RUN_TEST(test_exploracao_converge_ate_o_objetivo);
    return UNITY_END();
}
