#include "communication/Logger.h"

#include <cstdio>
#include <cstring>

namespace mm {

const char *logLevelToString(LogLevel level) {
    switch (level) {
        case LogLevel::Info:
            return "INFO";
        case LogLevel::Warn:
            return "WARN";
        case LogLevel::Error:
            return "ERROR";
        case LogLevel::Success:
            return "SUCCESS";
    }
    return "INFO";
}

bool stringToLogLevel(const char *str, LogLevel &outLevel) {
    if (!str) return false;
    if (std::strcmp(str, "INFO") == 0) {
        outLevel = LogLevel::Info;
        return true;
    }
    if (std::strcmp(str, "WARN") == 0) {
        outLevel = LogLevel::Warn;
        return true;
    }
    if (std::strcmp(str, "ERROR") == 0) {
        outLevel = LogLevel::Error;
        return true;
    }
    if (std::strcmp(str, "SUCCESS") == 0) {
        outLevel = LogLevel::Success;
        return true;
    }
    return false;
}

size_t Logger::formatJson(char *buffer, size_t bufferSize, uint32_t sessionId,
                          LogLevel level, const char *message, uint32_t nowMs) {
    if (!buffer || bufferSize == 0 || !message) {
        return 0;
    }

    const char *lvlStr = logLevelToString(level);
    int written = std::snprintf(
        buffer,
        bufferSize,
        "{\"type\":\"log\",\"session_id\":%u,\"nivel\":\"%s\",\"mensagem\":\"%s\",\"t_ms\":%u}",
        sessionId,
        lvlStr,
        message,
        nowMs
    );

    if (written < 0 || static_cast<size_t>(written) >= bufferSize) {
        return 0; // Truncado ou erro
    }

    return static_cast<size_t>(written);
}

bool Logger::log(LogLevel level, const char *message, uint32_t nowMs) {
    if (!message || message[0] == '\0') {
        return false;
    }

    char payload[kMaxPayloadSize];
    size_t len = formatJson(payload, sizeof(payload), session_id_, level, message, nowMs);
    if (len == 0) {
        return false;
    }

    return transport_.sendLog(payload, len);
}

}  // namespace mm
