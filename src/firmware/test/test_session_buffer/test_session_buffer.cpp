// Teste nativo da HU-11 (SessionBuffer).  pio test -e native -f test_session_buffer
#include <unity.h>

#include <cstring>
#include <string>

#include "hal/ISessionStorage.h"
#include "navigation/SessionBuffer.h"

using namespace mm;

namespace {

/* Fake em memória do HAL de fallback. */
class FakeStorage : public ISessionStorage {
public:
    std::string content;
    uint32_t    id = 0;
    bool fail_begin = false, fail_write = false, fail_commit = false;
    bool committed = false, aborted = false;

    bool begin(uint32_t session_id) override {
        if (fail_begin) return false;
        id = session_id; content.clear(); committed = aborted = false;
        return true;
    }
    bool write(const char *c, size_t n) override {
        if (fail_write) return false;
        content.append(c, n);
        return true;
    }
    bool commit() override { committed = !fail_commit; return committed; }
    void abort() override { aborted = true; }
};

SessionBuffer sb;   // ~12 KB: fora da stack

int collect(const char *chunk, size_t len, void *ctx)
{
    static_cast<std::string *>(ctx)->append(chunk, len);
    return 0;
}

}  // namespace

void setUp(void) { sb = SessionBuffer(); }
void tearDown(void) {}

void test_ignora_registro_sem_sessao(void)
{
    TEST_ASSERT_FALSE(sb.record(10, 0, 0, 0));
    TEST_ASSERT_EQUAL(SessionState::Idle, sb.state());
}

void test_start_unico(void)
{
    TEST_ASSERT_TRUE(sb.start(7, 1000));
    TEST_ASSERT_FALSE(sb.start(8, 1000));
    TEST_ASSERT_EQUAL_UINT32(7, sb.sessionId());
}

void test_registra_pontos_com_tempo_relativo(void)
{
    sb.start(7, 1000);
    sb.record(1000, 0, 0, 1);
    sb.record(1450, 1, 0, 1);
    TEST_ASSERT_EQUAL_size_t(2, sb.count());
    TEST_ASSERT_EQUAL_UINT32(0, sb.point(0).t_ms);
    TEST_ASSERT_EQUAL_UINT32(450, sb.point(1).t_ms);
    TEST_ASSERT_EQUAL_INT16(1, sb.point(1).x);
    TEST_ASSERT_EQUAL_UINT8(1, sb.point(1).heading);
}

void test_calcula_tempo_total_ao_parar(void)
{
    sb.start(7, 1000);
    TEST_ASSERT_TRUE(sb.stop(6500, EndReason::FinishLine));
    TEST_ASSERT_EQUAL_UINT32(5500, sb.totalTimeMs());          // TempoFinal - TempoInicial
    TEST_ASSERT_EQUAL(SessionState::Closed, sb.state());
    TEST_ASSERT_FALSE(sb.record(7000, 0, 0, 0));               // fechada: ignora
    TEST_ASSERT_FALSE(sb.stop(8000, EndReason::Error));        // não fecha duas vezes
    TEST_ASSERT_EQUAL_UINT32(5500, sb.totalTimeMs());
}

void test_tempo_total_com_overflow_do_relogio(void)
{
    sb.start(9, 0xFFFFFF00u);
    sb.stop(0x100u, EndReason::ManualStop);
    TEST_ASSERT_EQUAL_UINT32(0x200u, sb.totalTimeMs());
}

void test_decimacao_quando_o_buffer_enche(void)
{
    sb.start(10, 0);
    for (uint32_t i = 0; i < 5000; i++) {
        sb.record(i, static_cast<int16_t>(i % 16), 0, 0);
    }
    TEST_ASSERT_LESS_OR_EQUAL(SessionBuffer::kCapacity, sb.count());
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32(4, sb.stride());
    TEST_ASSERT_EQUAL_UINT32(0, sb.point(0).t_ms);
    TEST_ASSERT_EQUAL_UINT32(sb.stride(), sb.point(1).t_ms);   // espaçamento uniforme
    // o último ponto cobre o fim da rota (não só o começo)
    TEST_ASSERT_GREATER_THAN_UINT32(4000, sb.point(sb.count() - 1).t_ms);
}

void test_json_do_pacote(void)
{
    sb.start(42, 100);
    sb.record(100, 0, 0, 1);
    sb.record(600, 1, 0, 1);
    sb.record(1100, 1, 1, 2);
    std::string out;
    TEST_ASSERT_FALSE(sb.writeJson(collect, &out));            // aberta: recusa
    sb.stop(5100, EndReason::FinishLine);
    TEST_ASSERT_TRUE(sb.writeJson(collect, &out));
    TEST_ASSERT_EQUAL_STRING(
        "{\"session_id\":42,\"tempo_total_ms\":5000,\"motivo\":\"CHEGADA\",\"tipo_labirinto\":\"desconhecido\",\"tentativa\":1,\"desafio_cumprido\":true,\"consumo_bateria_mah\":null,\"stride\":1,\"pontos\":["
        "{\"t\":0,\"x\":0,\"y\":0,\"heading\":\"EAST\"},"
        "{\"t\":500,\"x\":1,\"y\":0,\"heading\":\"EAST\"},"
        "{\"t\":1000,\"x\":1,\"y\":1,\"heading\":\"SOUTH\"}]}",
        out.c_str());
}

void test_save_fallback_commita(void)
{
    sb.start(5, 0);
    sb.record(0, 0, 0, 0);
    sb.stop(1000, EndReason::Error);

    FakeStorage st;
    TEST_ASSERT_TRUE(sb.saveFallback(st));
    TEST_ASSERT_EQUAL_UINT32(5, st.id);
    TEST_ASSERT_TRUE(st.committed);
    TEST_ASSERT_FALSE(st.aborted);
    TEST_ASSERT_NOT_NULL(std::strstr(st.content.c_str(), "\"motivo\":\"ERRO\""));
}

void test_save_fallback_trata_falhas(void)
{
    sb.start(5, 0);
    sb.stop(1000, EndReason::FinishLine);

    FakeStorage a; a.fail_begin = true;
    TEST_ASSERT_FALSE(sb.saveFallback(a));

    FakeStorage b; b.fail_write = true;
    TEST_ASSERT_FALSE(sb.saveFallback(b));
    TEST_ASSERT_TRUE(b.aborted);                               // não deixa arquivo pela metade

    FakeStorage c; c.fail_commit = true;
    TEST_ASSERT_FALSE(sb.saveFallback(c));
    TEST_ASSERT_TRUE(c.aborted);
}

void test_save_fallback_exige_sessao_fechada(void)
{
    sb.start(5, 0);
    FakeStorage st;
    TEST_ASSERT_FALSE(sb.saveFallback(st));
}

int main(int, char **)
{
    UNITY_BEGIN();
    RUN_TEST(test_ignora_registro_sem_sessao);
    RUN_TEST(test_start_unico);
    RUN_TEST(test_registra_pontos_com_tempo_relativo);
    RUN_TEST(test_calcula_tempo_total_ao_parar);
    RUN_TEST(test_tempo_total_com_overflow_do_relogio);
    RUN_TEST(test_decimacao_quando_o_buffer_enche);
    RUN_TEST(test_json_do_pacote);
    RUN_TEST(test_save_fallback_commita);
    RUN_TEST(test_save_fallback_trata_falhas);
    RUN_TEST(test_save_fallback_exige_sessao_fechada);
    return UNITY_END();
}
