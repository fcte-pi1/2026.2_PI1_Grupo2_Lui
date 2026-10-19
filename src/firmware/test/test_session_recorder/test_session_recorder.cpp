// Teste nativo da HU-11 (SessionRecorder + PositionTracker).
// pio test -e native -f test_session_recorder
#include <unity.h>

#include "navigation/SessionBuffer.h"
#include "navigation/SessionRecorder.h"

using namespace mm;
using tracking::GridPosition;
using tracking::Orientation;
using tracking::TrackingState;

namespace {

SessionBuffer sb;   // fora da stack

TrackingState st(int16_t row, int16_t col, Orientation o, uint32_t ts)
{
    TrackingState s;
    s.position.row = row;
    s.position.col = col;
    s.orientation  = o;
    s.timestampMs  = ts;
    return s;
}

}  // namespace

void setUp(void) { sb = SessionBuffer(); }
void tearDown(void) {}

void test_begin_grava_celula_de_partida(void)
{
    SessionRecorder rec(sb);
    TEST_ASSERT_TRUE(rec.begin(1, st(0, 0, Orientation::East, 1000), 1000));
    TEST_ASSERT_EQUAL_size_t(1, sb.count());
    TEST_ASSERT_EQUAL_UINT32(0, sb.point(0).t_ms);
    TEST_ASSERT_EQUAL_UINT8(1, sb.point(0).heading);   // East
}

void test_so_grava_quando_a_celula_muda(void)
{
    SessionRecorder rec(sb);
    rec.begin(1, st(0, 0, Orientation::East, 0), 0);

    TEST_ASSERT_FALSE(rec.onTracking(st(0, 0, Orientation::East, 50)));   // mesma célula
    TEST_ASSERT_FALSE(rec.onTracking(st(0, 0, Orientation::South, 80)));  // só girou
    TEST_ASSERT_TRUE(rec.onTracking(st(0, 1, Orientation::South, 300)));  // avançou
    TEST_ASSERT_FALSE(rec.onTracking(st(0, 1, Orientation::South, 320)));
    TEST_ASSERT_TRUE(rec.onTracking(st(1, 1, Orientation::South, 700)));

    TEST_ASSERT_EQUAL_size_t(3, sb.count());
    TEST_ASSERT_EQUAL_INT16(1, sb.point(1).x);    // x = col
    TEST_ASSERT_EQUAL_INT16(0, sb.point(1).y);    // y = row
    TEST_ASSERT_EQUAL_UINT32(300, sb.point(1).t_ms);
    TEST_ASSERT_EQUAL_INT16(1, sb.point(2).y);
}

void test_voltar_para_celula_anterior_tambem_grava(void)
{
    SessionRecorder rec(sb);
    rec.begin(1, st(0, 0, Orientation::East, 0), 0);
    rec.onTracking(st(0, 1, Orientation::East, 100));
    TEST_ASSERT_TRUE(rec.onTracking(st(0, 0, Orientation::West, 200)));   // retorno
    TEST_ASSERT_EQUAL_size_t(3, sb.count());
}

void test_timestamp_atrasado_nao_gera_tempo_negativo(void)
{
    SessionRecorder rec(sb);
    rec.begin(1, st(0, 0, Orientation::East, 5000), 5000);
    rec.onTracking(st(0, 1, Orientation::East, 4000));   // anterior ao início
    TEST_ASSERT_EQUAL_UINT32(0, sb.point(1).t_ms);
}

void test_nao_grava_sem_sessao_ou_depois_de_fechar(void)
{
    SessionRecorder rec(sb);
    TEST_ASSERT_FALSE(rec.onTracking(st(0, 1, Orientation::East, 10)));
    rec.begin(1, st(0, 0, Orientation::East, 0), 0);
    TEST_ASSERT_TRUE(rec.running());
    TEST_ASSERT_TRUE(rec.end(9000, EndReason::FinishLine));
    TEST_ASSERT_FALSE(rec.running());
    TEST_ASSERT_FALSE(rec.onTracking(st(0, 2, Orientation::East, 9100)));
    TEST_ASSERT_EQUAL_UINT32(9000, sb.totalTimeMs());
}

void test_begin_recusado_com_sessao_em_andamento(void)
{
    SessionRecorder rec(sb);
    TEST_ASSERT_TRUE(rec.begin(1, st(0, 0, Orientation::East, 0), 0));
    TEST_ASSERT_FALSE(rec.begin(2, st(0, 0, Orientation::East, 0), 0));
    TEST_ASSERT_EQUAL_UINT32(1, sb.sessionId());
}

int main(int, char **)
{
    UNITY_BEGIN();
    RUN_TEST(test_begin_grava_celula_de_partida);
    RUN_TEST(test_so_grava_quando_a_celula_muda);
    RUN_TEST(test_voltar_para_celula_anterior_tambem_grava);
    RUN_TEST(test_timestamp_atrasado_nao_gera_tempo_negativo);
    RUN_TEST(test_nao_grava_sem_sessao_ou_depois_de_fechar);
    RUN_TEST(test_begin_recusado_com_sessao_em_andamento);
    return UNITY_END();
}
