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
    if (!buffer || bufferSize == 0 || !message || sessionId == 0) {
        return 0;
    }

    int prefixLength = std::snprintf(
        buffer, bufferSize,
        "{\"type\":\"log\",\"session_id\":%u,\"nivel\":\"%s\",\"mensagem\":\"",
        sessionId, logLevelToString(level));
    if (prefixLength < 0 || static_cast<size_t>(prefixLength) >= bufferSize) {
        return 0;
    }

    size_t pos = static_cast<size_t>(prefixLength);
    static const char hex[] = "0123456789abcdef";
    for (const unsigned char *cursor = reinterpret_cast<const unsigned char *>(message);
         *cursor != '\0'; ++cursor) {
        char escaped[6];
        size_t escapedLength = 1;
        escaped[0] = static_cast<char>(*cursor);
        switch (*cursor) {
            case '"': escaped[0] = '\\'; escaped[1] = '"'; escapedLength = 2; break;
            case '\\': escaped[0] = '\\'; escaped[1] = '\\'; escapedLength = 2; break;
            case '\b': escaped[0] = '\\'; escaped[1] = 'b'; escapedLength = 2; break;
            case '\f': escaped[0] = '\\'; escaped[1] = 'f'; escapedLength = 2; break;
            case '\n': escaped[0] = '\\'; escaped[1] = 'n'; escapedLength = 2; break;
            case '\r': escaped[0] = '\\'; escaped[1] = 'r'; escapedLength = 2; break;
            case '\t': escaped[0] = '\\'; escaped[1] = 't'; escapedLength = 2; break;
            default:
                if (*cursor < 0x20) {
                    escaped[0] = '\\'; escaped[1] = 'u'; escaped[2] = '0'; escaped[3] = '0';
                    escaped[4] = hex[*cursor >> 4]; escaped[5] = hex[*cursor & 0x0f];
                    escapedLength = 6;
                }
                break;
        }
        if (pos + escapedLength + 1 >= bufferSize) {
            buffer[0] = '\0';
            return 0;
        }
        for (size_t i = 0; i < escapedLength; ++i) buffer[pos++] = escaped[i];
    }

    int suffixLength = std::snprintf(buffer + pos, bufferSize - pos,
                                     "\",\"t_ms\":%u}", nowMs);
    if (suffixLength < 0 || static_cast<size_t>(suffixLength) >= bufferSize - pos) {
        buffer[0] = '\0';
        return 0;
    }
    return pos + static_cast<size_t>(suffixLength);
}

bool Logger::log(LogLevel level, const char *message, uint32_t nowMs) {
    if (session_id_ == 0 || !message || message[0] == '\0') {
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
