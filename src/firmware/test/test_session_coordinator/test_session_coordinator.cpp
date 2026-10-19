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

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_mesmo_id_liga_pontos_e_logs);
    RUN_TEST(test_falha_no_transporte_nao_impede_fechar_sessao);
    return UNITY_END();
}
