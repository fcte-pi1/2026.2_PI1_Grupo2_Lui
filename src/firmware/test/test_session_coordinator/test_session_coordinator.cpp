// Integração entre histórico de navegação (HU-11) e diagnósticos (HU-12).
#include <unity.h>

#include <cstring>
#include <string>

#include "communication/Logger.h"
#include "navigation/SessionCoordinator.h"
#include "../test_logger/mock_log_transport.h"

using namespace mm;
using mm::test::MockLogTransport;

namespace {
SessionBuffer buffer; // grande: manter fora da stack

SessionPose pose(int16_t row, int16_t col, uint8_t heading, uint32_t t) {
    return SessionPose{row, col, heading, t};
}
}

void setUp(void) { buffer = SessionBuffer(); }
void tearDown(void) {}

void test_mesmo_id_liga_pontos_e_logs(void) {
    MockLogTransport transport;
    Logger logger(transport);
    SessionCoordinator coordinator(buffer, logger);

    TEST_ASSERT_TRUE(coordinator.begin(123, pose(0, 0, 1, 500), 500, "8x4", 2));
    TEST_ASSERT_EQUAL_UINT32(123, logger.sessionId());
    TEST_ASSERT_EQUAL_UINT32(1, buffer.count());
    TEST_ASSERT_EQUAL_UINT32(1, transport.sentMessages.size());
    TEST_ASSERT_NOT_NULL(std::strstr(transport.sentMessages[0].c_str(), "\"session_id\":123"));

    TEST_ASSERT_TRUE(coordinator.onTracking(pose(0, 1, 1, 800)));
    TEST_ASSERT_TRUE(coordinator.finish(1800, EndReason::FinishLine));
    TEST_ASSERT_EQUAL_UINT32(2, buffer.count());
    TEST_ASSERT_EQUAL_UINT32(2, transport.sentMessages.size());
    TEST_ASSERT_NOT_NULL(std::strstr(transport.sentMessages[1].c_str(), "\"nivel\":\"SUCCESS\""));
    TEST_ASSERT_NOT_NULL(std::strstr(transport.sentMessages[1].c_str(), "\"t_ms\":1300"));
    TEST_ASSERT_EQUAL(SessionState::Closed, coordinator.session().state());
}

void test_falha_no_transporte_nao_impede_fechar_sessao(void) {
    MockLogTransport transport;
    transport.shouldFail = true;
    Logger logger(transport);
    SessionCoordinator coordinator(buffer, logger);
    TEST_ASSERT_TRUE(coordinator.begin(7, pose(0, 0, 0, 0), 0, "4x4", 1));
    TEST_ASSERT_TRUE(coordinator.finish(50, EndReason::Error));
    TEST_ASSERT_EQUAL(SessionState::Closed, buffer.state());
}


void test_inicio_e_fim_rejeitados_quando_estado_da_sessao_nao_permite(void) {
    MockLogTransport transport;
    Logger logger(transport);
    SessionCoordinator coordinator(buffer, logger);

    // Não se pode encerrar uma sessão que ainda não foi iniciada.
    TEST_ASSERT_FALSE(coordinator.finish(10, EndReason::ManualStop));
    // ID zero é rejeitado pelo buffer, sem definir o ID do logger.
    TEST_ASSERT_FALSE(coordinator.begin(0, pose(0, 0, 0, 0), 0, "4x4", 1));
    TEST_ASSERT_EQUAL_UINT32(0, logger.sessionId());

    TEST_ASSERT_TRUE(coordinator.begin(8, pose(0, 0, 0, 10), 10, "4x4", 1));
    TEST_ASSERT_FALSE(coordinator.begin(9, pose(0, 0, 0, 20), 20, "4x4", 1));
    TEST_ASSERT_TRUE(coordinator.finish(30, EndReason::ManualStop));
    TEST_ASSERT_EQUAL(SessionState::Closed, coordinator.session().state());
    TEST_ASSERT_EQUAL_UINT32(2, transport.sentMessages.size());
    TEST_ASSERT_NOT_NULL(std::strstr(transport.sentMessages[1].c_str(), "\"nivel\":\"WARN\""));
    // Uma segunda finalização é recusada e não cria um terceiro log.
    TEST_ASSERT_FALSE(coordinator.finish(40, EndReason::Error));
    TEST_ASSERT_EQUAL_UINT32(2, transport.sentMessages.size());
}

void test_finalizacao_por_linha_de_chegada_e_erro(void) {
    for (EndReason reason : {EndReason::FinishLine, EndReason::Error}) {
        buffer = SessionBuffer();
        MockLogTransport transport;
        Logger logger(transport);
        SessionCoordinator coordinator(buffer, logger);
        TEST_ASSERT_TRUE(coordinator.begin(11, pose(0, 0, 1, 0), 0, "8x4", 1));
        TEST_ASSERT_TRUE(coordinator.finish(100, reason));
        TEST_ASSERT_EQUAL_UINT32(2, transport.sentMessages.size());
        if (reason == EndReason::FinishLine) {
            TEST_ASSERT_NOT_NULL(std::strstr(transport.sentMessages[1].c_str(), "\"nivel\":\"SUCCESS\""));
        } else {
            TEST_ASSERT_NOT_NULL(std::strstr(transport.sentMessages[1].c_str(), "\"nivel\":\"ERROR\""));
        }
    }
}


void test_motivo_desconhecido_nao_causa_queda(void) {
    MockLogTransport transport;
    Logger logger(transport);
    SessionCoordinator coordinator(buffer, logger);
    TEST_ASSERT_TRUE(coordinator.begin(20, pose(0, 0, 0, 0), 0, "4x4", 1));

    // O contrato normal só usa os três enumeradores conhecidos. Este caso
    // cobre a saída defensiva do switch quando chega um valor corrompido.
    const EndReason unknown = static_cast<EndReason>(99);
    TEST_ASSERT_TRUE(coordinator.finish(25, unknown));
    TEST_ASSERT_EQUAL(SessionState::Closed, coordinator.session().state());
    TEST_ASSERT_EQUAL_UINT32(1, transport.sentMessages.size()); // só o log de início
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_mesmo_id_liga_pontos_e_logs);
    RUN_TEST(test_falha_no_transporte_nao_impede_fechar_sessao);
    RUN_TEST(test_inicio_e_fim_rejeitados_quando_estado_da_sessao_nao_permite);
    RUN_TEST(test_finalizacao_por_linha_de_chegada_e_erro);
    RUN_TEST(test_motivo_desconhecido_nao_causa_queda);
    return UNITY_END();
}
