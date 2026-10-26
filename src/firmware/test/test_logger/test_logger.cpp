#include <unity.h>

#include <cstring>
#include <string>

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


// Conversão do texto para nível: valores reconhecidos, nulos e desconhecidos.
static void test_string_to_log_level_all_paths(void) {
    LogLevel level = LogLevel::Info;
    TEST_ASSERT_FALSE(stringToLogLevel(nullptr, level));

    TEST_ASSERT_TRUE(stringToLogLevel("INFO", level));
    TEST_ASSERT_EQUAL(static_cast<int>(LogLevel::Info), static_cast<int>(level));
    TEST_ASSERT_TRUE(stringToLogLevel("WARN", level));
    TEST_ASSERT_EQUAL(static_cast<int>(LogLevel::Warn), static_cast<int>(level));
    TEST_ASSERT_TRUE(stringToLogLevel("ERROR", level));
    TEST_ASSERT_EQUAL(static_cast<int>(LogLevel::Error), static_cast<int>(level));
    TEST_ASSERT_TRUE(stringToLogLevel("SUCCESS", level));
    TEST_ASSERT_EQUAL(static_cast<int>(LogLevel::Success), static_cast<int>(level));
    TEST_ASSERT_FALSE(stringToLogLevel("DEBUG", level));
}

// Cobre as validações do formatador e os dois caminhos de buffer insuficiente.
static void test_format_json_rejects_invalid_arguments_and_small_buffers(void) {
    char buffer[128] = "conteudo anterior";
    TEST_ASSERT_EQUAL_UINT32(0, Logger::formatJson(nullptr, sizeof(buffer), 1,
                                                   LogLevel::Info, "x", 0));
    TEST_ASSERT_EQUAL_UINT32(0, Logger::formatJson(buffer, 0, 1,
                                                   LogLevel::Info, "x", 0));
    TEST_ASSERT_EQUAL_UINT32(0, Logger::formatJson(buffer, sizeof(buffer), 1,
                                                   LogLevel::Info, nullptr, 0));
    TEST_ASSERT_EQUAL_UINT32(0, Logger::formatJson(buffer, sizeof(buffer), 0,
                                                   LogLevel::Info, "x", 0));

    char prefixTooSmall[1] = {'x'};
    TEST_ASSERT_EQUAL_UINT32(0, Logger::formatJson(prefixTooSmall, sizeof(prefixTooSmall),
                                                   1, LogLevel::Info, "", 0));

    const char *prefix = "{\"type\":\"log\",\"session_id\":1,\"nivel\":\"INFO\",\"mensagem\":\"";
    const size_t prefixLength = std::strlen(prefix);
    // O prefixo cabe, mas não há espaço para o sufixo e o terminador.
    TEST_ASSERT_EQUAL_UINT32(0, Logger::formatJson(buffer, prefixLength + 2,
                                                   1, LogLevel::Info, "", 0));
    TEST_ASSERT_EQUAL_CHAR('\0', buffer[0]);

    MockLogTransport mock;
    Logger logger(mock);
    logger.setSessionId(1);
    const std::string oversized(Logger::kMaxPayloadSize + 32, 'x');
    TEST_ASSERT_FALSE(logger.log(LogLevel::Info, oversized.c_str(), 1));
    TEST_ASSERT_EQUAL_UINT32(0, mock.sentMessages.size());
}

// Todos os caracteres de controle que o formatador escapa, incluindo o caso \u00XX.
static void test_format_json_escapes_every_control_character(void) {
    const char message[] = {'"', '\\', '\b', '\f', '\n', '\r', '\t',
                            static_cast<char>(0x01), static_cast<char>(0x1f), 'A', '\0'};
    char buffer[256];
    const size_t length = Logger::formatJson(buffer, sizeof(buffer), 3,
                                              LogLevel::Info, message, 9);
    TEST_ASSERT_GREATER_THAN(0, length);
    TEST_ASSERT_EQUAL_STRING(
        "{\"type\":\"log\",\"session_id\":3,\"nivel\":\"INFO\",\"mensagem\":\"\\\"\\\\\\b\\f\\n\\r\\t\\u0001\\u001fA\",\"t_ms\":9}",
        buffer);
}

static void test_logger_rejects_missing_session_and_unknown_log_level(void) {
    MockLogTransport mock;
    Logger logger(mock);
    TEST_ASSERT_EQUAL_STRING("INFO", logLevelToString(static_cast<LogLevel>(99)));
    TEST_ASSERT_FALSE(logger.info("sem sessao", 1));
    TEST_ASSERT_EQUAL_UINT32(0, mock.callCount);
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
    RUN_TEST(test_string_to_log_level_all_paths);
    RUN_TEST(test_format_json_rejects_invalid_arguments_and_small_buffers);
    RUN_TEST(test_format_json_escapes_every_control_character);
    RUN_TEST(test_logger_rejects_missing_session_and_unknown_log_level);
    return UNITY_END();
}
