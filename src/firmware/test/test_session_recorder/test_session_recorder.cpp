// Teste nativo da HU-11 (SessionRecorder + PositionTracker).
// pio test -e native -f test_session_recorder
#include <unity.h>

#include <cstring>
#include <string>

#include "navigation/SessionBuffer.h"
#include "navigation/SessionRecorder.h"

using namespace mm;

namespace {

SessionBuffer sb;   // fora da stack

int collect(const char *chunk, size_t len, void *ctx)
{
    static_cast<std::string *>(ctx)->append(chunk, len);
    return 0;
}

// Estrutura compatível com a pose do PositionTracker (HU-02), sem exigir que
// os arquivos de outra HU estejam no mesmo branch para testar a HU-11.
struct TestTrackingState {
    struct Position { int16_t row; int16_t col; } position;
    uint8_t orientation;
    uint32_t timestampMs;
};

TestTrackingState st(int16_t row, int16_t col, uint8_t heading, uint32_t ts)
{
    return {{row, col}, heading, ts};
}

}  // namespace

void setUp(void) { sb = SessionBuffer(); }
void tearDown(void) {}

void test_begin_grava_celula_de_partida(void)
{
    SessionRecorder rec(sb);
    TEST_ASSERT_TRUE(rec.begin(1, st(0, 0, 1, 1000), 1000));
    TEST_ASSERT_EQUAL_size_t(1, sb.count());
    TEST_ASSERT_EQUAL_UINT32(0, sb.point(0).t_ms);
    TEST_ASSERT_EQUAL_UINT8(1, sb.point(0).heading);   // East
}

void test_so_grava_quando_a_celula_muda(void)
{
    SessionRecorder rec(sb);
    rec.begin(1, st(0, 0, 1, 0), 0);

    TEST_ASSERT_FALSE(rec.onTracking(st(0, 0, 1, 50)));   // mesma célula
    TEST_ASSERT_FALSE(rec.onTracking(st(0, 0, 2, 80)));  // só girou
    TEST_ASSERT_TRUE(rec.onTracking(st(0, 1, 2, 300)));  // avançou
    TEST_ASSERT_FALSE(rec.onTracking(st(0, 1, 2, 320)));
    TEST_ASSERT_TRUE(rec.onTracking(st(1, 1, 2, 700)));

    TEST_ASSERT_EQUAL_size_t(3, sb.count());
    TEST_ASSERT_EQUAL_INT16(1, sb.point(1).x);    // x = col
    TEST_ASSERT_EQUAL_INT16(0, sb.point(1).y);    // y = row
    TEST_ASSERT_EQUAL_UINT32(300, sb.point(1).t_ms);
    TEST_ASSERT_EQUAL_INT16(1, sb.point(2).y);
}

void test_voltar_para_celula_anterior_tambem_grava(void)
{
    SessionRecorder rec(sb);
    rec.begin(1, st(0, 0, 1, 0), 0);
    rec.onTracking(st(0, 1, 1, 100));
    TEST_ASSERT_TRUE(rec.onTracking(st(0, 0, 3, 200)));   // retorno
    TEST_ASSERT_EQUAL_size_t(3, sb.count());
}

void test_timestamp_atrasado_nao_gera_tempo_negativo(void)
{
    SessionRecorder rec(sb);
    rec.begin(1, st(0, 0, 1, 5000), 5000);
    rec.onTracking(st(0, 1, 1, 4000));   // anterior ao início
    TEST_ASSERT_EQUAL_UINT32(0, sb.point(1).t_ms);
}

void test_nao_grava_sem_sessao_ou_depois_de_fechar(void)
{
    SessionRecorder rec(sb);
    TEST_ASSERT_FALSE(rec.onTracking(st(0, 1, 1, 10)));
    rec.begin(1, st(0, 0, 1, 0), 0);
    TEST_ASSERT_TRUE(rec.running());
    TEST_ASSERT_TRUE(rec.end(9000, EndReason::FinishLine));
    TEST_ASSERT_FALSE(rec.running());
    TEST_ASSERT_FALSE(rec.onTracking(st(0, 2, 1, 9100)));
    TEST_ASSERT_EQUAL_UINT32(9000, sb.totalTimeMs());
}

void test_begin_recusado_com_sessao_em_andamento(void)
{
    SessionRecorder rec(sb);
    TEST_ASSERT_TRUE(rec.begin(1, st(0, 0, 1, 0), 0));
    TEST_ASSERT_FALSE(rec.begin(2, st(0, 0, 1, 0), 0));
    TEST_ASSERT_EQUAL_UINT32(1, sb.sessionId());
}

void test_json_conserva_metadados_hu11(void)
{
    sb.start(77, 10, "12x4", 3);
    sb.record(10, 0, 0, 0);
    sb.setBatteryConsumedMah(12.5f);
    sb.stop(1110, EndReason::ManualStop);
    std::string out;
    TEST_ASSERT_TRUE(sb.writeJson(collect, &out));
    TEST_ASSERT_NOT_NULL(std::strstr(out.c_str(), "\"tipo_labirinto\":\"12x4\""));
    TEST_ASSERT_NOT_NULL(std::strstr(out.c_str(), "\"tentativa\":3"));
    TEST_ASSERT_NOT_NULL(std::strstr(out.c_str(), "\"desafio_cumprido\":false"));
    TEST_ASSERT_NOT_NULL(std::strstr(out.c_str(), "\"consumo_bateria_mah\":12.500"));
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
    RUN_TEST(test_json_conserva_metadados_hu11);
    return UNITY_END();
}
