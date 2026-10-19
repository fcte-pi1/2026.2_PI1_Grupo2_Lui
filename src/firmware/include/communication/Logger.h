#pragma once

#include <cstddef>
#include <cstdint>

#include "communication/ILogTransport.h"

namespace mm {

/**
 * Níveis de severidade do log de eventos (HU-12).
 * Segue o padrão definido na arquitetura e aceito pelo backend:
 * INFO, WARN, ERROR, SUCCESS.
 */
enum class LogLevel : uint8_t {
    Info,
    Warn,
    Error,
    Success
};

/**
 * Converte o enum de severidade para sua representação em texto ("INFO", "WARN", etc.).
 */
const char *logLevelToString(LogLevel level);

/**
 * Converte texto para enum de severidade. Retorna false se o texto for desconhecido.
 */
bool stringToLogLevel(const char *str, LogLevel &outLevel);

/**
 * Módulo de Logger (HU-12).
 *
 * Formata eventos no padrão JSON esperado pelo backend:
 * {"type":"log","session_id":<id>,"nivel":"<NIVEL>","mensagem":"<msg>","t_ms":<t>}
 *
 * Lógica pura: pode ser testada nativamente (sem ESP32).
 */
class Logger {
public:
    static constexpr size_t kMaxPayloadSize = 256;

    explicit Logger(ILogTransport &transport) : transport_(transport) {}

    void setSessionId(uint32_t sessionId) { session_id_ = sessionId; }
    uint32_t sessionId() const { return session_id_; }

    /**
     * Registra um evento com nível, mensagem e timestamp (ms desde o início da sessão).
     */
    bool log(LogLevel level, const char *message, uint32_t nowMs);

    bool info(const char *message, uint32_t nowMs) {
        return log(LogLevel::Info, message, nowMs);
    }

    bool warn(const char *message, uint32_t nowMs) {
        return log(LogLevel::Warn, message, nowMs);
    }

    bool error(const char *message, uint32_t nowMs) {
        return log(LogLevel::Error, message, nowMs);
    }

    bool success(const char *message, uint32_t nowMs) {
        return log(LogLevel::Success, message, nowMs);
    }

    /**
     * Formata o JSON no buffer fornecido.
     * Retorna o número de bytes gravados (excluindo null terminator) ou 0 se estourar o buffer.
     */
    static size_t formatJson(char *buffer, size_t bufferSize, uint32_t sessionId,
                             LogLevel level, const char *message, uint32_t nowMs);

private:
    ILogTransport &transport_;
    uint32_t session_id_ = 0;
};

}  // namespace mm
