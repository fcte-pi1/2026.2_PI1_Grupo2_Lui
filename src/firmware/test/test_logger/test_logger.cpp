#include <unity.h>

#include "communication/Logger.h"
#include "mock_log_transport.h"

using namespace mm;
using mm::test::MockLogTransport;

void setUp(void) {}
void tearDown(void) {}

// Teste de sanidade: mock registra chamadas e falhas programadas
static void test_mock_transport_behavior(void) {
    MockLogTransport mock;
    TEST_ASSERT_TRUE(mock.sendLog("teste", 5));
    TEST_ASSERT_EQUAL_UINT32(1, mock.callCount);
    TEST_ASSERT_EQUAL_STRING("teste", mock.sentMessages[0].c_str());

    mock.shouldFail = true;
    TEST_ASSERT_FALSE(mock.sendLog("outro", 5));
    TEST_ASSERT_EQUAL_UINT32(2, mock.callCount);
    TEST_ASSERT_EQUAL_UINT32(1, mock.sentMessages.size());
}

// Conversão de níveis para texto
static void test_log_level_to_string(void) {
    TEST_ASSERT_EQUAL_STRING("INFO", logLevelToString(LogLevel::Info));
    TEST_ASSERT_EQUAL_STRING("WARN", logLevelToString(LogLevel::Warn));
    TEST_ASSERT_EQUAL_STRING("ERROR", logLevelToString(LogLevel::Error));
    TEST_ASSERT_EQUAL_STRING("SUCCESS", logLevelToString(LogLevel::Success));
}

// Formatação JSON válida com todos os campos esperados pelo backend
static void test_format_json_structure(void) {
    char buf[128];
    size_t len = Logger::formatJson(buf, sizeof(buf), 42, LogLevel::Warn, "Derrapagem", 1500);

    TEST_ASSERT_GREATER_THAN(0, len);
    TEST_ASSERT_EQUAL_STRING(
        "{\"type\":\"log\",\"session_id\":42,\"nivel\":\"WARN\",\"mensagem\":\"Derrapagem\",\"t_ms\":1500}",
        buf
    );
}

// Envio de log com os métodos utilitários (info, warn, error, success)
static void test_logger_helper_methods(void) {
    MockLogTransport mock;
    Logger logger(mock);
    logger.setSessionId(10);

    TEST_ASSERT_TRUE(logger.info("Sistema iniciado", 0));
    TEST_ASSERT_TRUE(logger.warn("Sensor com ruido", 500));
    TEST_ASSERT_TRUE(logger.error("Colisao detectada", 1200));
    TEST_ASSERT_TRUE(logger.success("Chegada alcancada", 3000));

    TEST_ASSERT_EQUAL_UINT32(4, mock.sentMessages.size());
    TEST_ASSERT_EQUAL_STRING(
        "{\"type\":\"log\",\"session_id\":10,\"nivel\":\"INFO\",\"mensagem\":\"Sistema iniciado\",\"t_ms\":0}",
        mock.sentMessages[0].c_str()
    );
    TEST_ASSERT_EQUAL_STRING(
        "{\"type\":\"log\",\"session_id\":10,\"nivel\":\"WARN\",\"mensagem\":\"Sensor com ruido\",\"t_ms\":500}",
        mock.sentMessages[1].c_str()
    );
    TEST_ASSERT_EQUAL_STRING(
        "{\"type\":\"log\",\"session_id\":10,\"nivel\":\"ERROR\",\"mensagem\":\"Colisao detectada\",\"t_ms\":1200}",
        mock.sentMessages[2].c_str()
    );
    TEST_ASSERT_EQUAL_STRING(
        "{\"type\":\"log\",\"session_id\":10,\"nivel\":\"SUCCESS\",\"mensagem\":\"Chegada alcancada\",\"t_ms\":3000}",
        mock.sentMessages[3].c_str()
    );
}

// Rejeição de mensagem nula ou vazia
static void test_reject_empty_or_null_message(void) {
    MockLogTransport mock;
    Logger logger(mock);
    logger.setSessionId(1);

    TEST_ASSERT_FALSE(logger.log(LogLevel::Info, nullptr, 100));
    TEST_ASSERT_FALSE(logger.log(LogLevel::Info, "", 100));
    TEST_ASSERT_EQUAL_UINT32(0, mock.sentMessages.size());
}

// Mensagens com aspas e quebras de linha continuam sendo JSON válido.
static void test_escapa_mensagem_json(void) {
    char buf[128];
    size_t len = Logger::formatJson(buf, sizeof(buf), 7, LogLevel::Error,
                                    "Falha: \"sensor\"\nreiniciado", 55);
    TEST_ASSERT_GREATER_THAN(0, len);
    TEST_ASSERT_EQUAL_STRING(
        "{\"type\":\"log\",\"session_id\":7,\"nivel\":\"ERROR\",\"mensagem\":\"Falha: \\\"sensor\\\"\\nreiniciado\",\"t_ms\":55}",
        buf);
}

// Propagação de falha do transporte
static void test_transport_failure_propagates(void) {
    MockLogTransport mock;
    mock.shouldFail = true;
    Logger logger(mock);
    logger.setSessionId(1);

    TEST_ASSERT_FALSE(logger.info("Msg", 100));
    TEST_ASSERT_EQUAL_UINT32(1, mock.callCount);
    TEST_ASSERT_EQUAL_UINT32(0, mock.sentMessages.size());
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_mock_transport_behavior);
    RUN_TEST(test_log_level_to_string);
    RUN_TEST(test_format_json_structure);
    RUN_TEST(test_logger_helper_methods);
    RUN_TEST(test_reject_empty_or_null_message);
    RUN_TEST(test_escapa_mensagem_json);
    RUN_TEST(test_transport_failure_propagates);
    return UNITY_END();
}
