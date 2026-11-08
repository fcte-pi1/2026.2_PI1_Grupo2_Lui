#ifdef ARDUINO

#include "communication/TelemetryClient.h"

#include <Arduino.h>
#include <cstring>

#include "config/network_config.hpp"

namespace mm {
namespace {
constexpr UBaseType_t QUEUE_LENGTH = 16;
constexpr uint32_t SEND_TIMEOUT_MS = 2000;

struct StringWriterContext {
    String* output;
    bool ok;
};

int appendSessionChunk(const char* chunk, size_t length, void* context) {
    auto* writer = static_cast<StringWriterContext*>(context);
    if (writer == nullptr || writer->output == nullptr || !writer->ok) return -1;
    writer->ok = writer->output->concat(chunk, length);
    return writer->ok ? 0 : -1;
}
}  // namespace

bool TelemetryClient::begin() {
    if (queue_ != nullptr) return true;

    queue_ = xQueueCreate(QUEUE_LENGTH, sizeof(QueueItem));
    if (queue_ == nullptr) return false;

    if (!bluetooth_.begin(MM_BT_DEVICE_NAME)) {
        Serial.println("HU-10: falha ao inicializar o servidor Bluetooth Classic SPP.");
        vQueueDelete(queue_);
        queue_ = nullptr;
        return false;
    }

    if (xTaskCreatePinnedToCore(taskEntry, "telemetry-bt", 8192, this, 1, &task_, 0) != pdPASS) {
        bluetooth_.end();
        vQueueDelete(queue_);
        queue_ = nullptr;
        task_ = nullptr;
        return false;
    }

    Serial.printf("HU-10: Bluetooth Classic SPP anunciado como '%s'; aguardando pareamento.\n",
                  MM_BT_DEVICE_NAME);
    return true;
}

bool TelemetryClient::publishJson(const char* json, size_t length) {
    return enqueue(ItemKind::Json, json, length);
}

bool TelemetryClient::publishMazeConfig(const char* json, size_t length,
                                        uint8_t width, uint8_t height) {
    if (json == nullptr || length == 0 || length >= sizeof(mazePayload_) ||
        width == 0 || width > kMaxMazeWidth || height == 0 || height > kMaxMazeHeight) {
        ++droppedMessages_;
        return false;
    }
    portENTER_CRITICAL(&stateMux_);
    memcpy(mazePayload_, json, length);
    mazePayload_[length] = '\0';
    mazeLength_ = length;
    mazeWidth_ = width;
    mazeHeight_ = height;
    portEXIT_CRITICAL(&stateMux_);
    // Enfileira imediatamente; em caso de perda, o cache será replayado na reconexão.
    return enqueue(ItemKind::Json, json, length);
}

bool TelemetryClient::publishMapSnapshot(uint8_t cellX, uint8_t cellY,
                                         const char* json, size_t length) {
    if (json == nullptr || length == 0 || length >= sizeof(latestCellPayload_)) {
        ++droppedMessages_;
        return false;
    }
    portENTER_CRITICAL(&stateMux_);
    if (cellX >= mazeWidth_ || cellY >= mazeHeight_) {
        portEXIT_CRITICAL(&stateMux_);
        ++droppedMessages_;
        return false;
    }
    const size_t index = static_cast<size_t>(cellY) * kMaxMazeWidth + cellX;
    memcpy(cellPayloads_[index], json, length);
    cellPayloads_[index][length] = '\0';
    cellLengths_[index] = static_cast<uint16_t>(length);
    cellValid_[index] = true;
    memcpy(latestCellPayload_, json, length);
    latestCellPayload_[length] = '\0';
    latestCellLength_ = length;
    portEXIT_CRITICAL(&stateMux_);
    return enqueue(ItemKind::Json, json, length);
}

bool TelemetryClient::publishSessionStart(const char* json, size_t length) {
    if (json == nullptr || length == 0 || length >= sizeof(sessionStartPayload_)) {
        ++droppedMessages_;
        return false;
    }
    portENTER_CRITICAL(&stateMux_);
    memcpy(sessionStartPayload_, json, length);
    sessionStartPayload_[length] = '\0';
    sessionStartLength_ = length;
    sessionStartPending_ = true;
    sessionActive_ = true;
    sessionClosedLocally_ = false;
    portEXIT_CRITICAL(&stateMux_);
    return true;
}

bool TelemetryClient::publishTelemetry(const char* json, size_t length) {
    return enqueue(ItemKind::Telemetry, json, length);
}

bool TelemetryClient::publishSessionEnd(const SessionBuffer& session) {
    if (session.state() != SessionState::Closed) return false;
    portENTER_CRITICAL(&stateMux_);
    pendingSessionEnd_ = &session;
    sessionClosedLocally_ = true;
    sessionActive_ = false;
    sessionStartPending_ = false;
    portEXIT_CRITICAL(&stateMux_);
    // Não entra na fila transitória: é reenviado após a reconexão até sucesso.
    return true;
}

bool TelemetryClient::sendLog(const char* payload, size_t length) {
    return publishJson(payload, length);
}

bool TelemetryClient::enqueue(ItemKind kind, const char* json, size_t length) {
    QueueItem item{};
    if (queue_ == nullptr || json == nullptr || length == 0 || length >= sizeof(item.payload)) {
        ++droppedMessages_;
        return false;
    }
    item.kind = kind;
    item.queuedAtMs = millis();
    memcpy(item.payload, json, length);
    item.payload[length] = '\0';
    if (xQueueSend(queue_, &item, 0) != pdTRUE) {
        ++droppedMessages_;
        return false;
    }
    return true;
}

void TelemetryClient::taskEntry(void* context) {
    auto* self = static_cast<TelemetryClient*>(context);
    if (self != nullptr) self->run();
    vTaskDelete(nullptr);
}

bool TelemetryClient::sendFrame(const char* payload, size_t length) {
    if (payload == nullptr || length == 0 || !bluetooth_.hasClient()) return false;

    // BluetoothSerial enfileira um frame inteiro ou retorna zero quando a fila
    // SPP está congestionada. Aguarda somente nesta task de baixa prioridade.
    const uint32_t startedAt = millis();
    size_t written = 0;
    while (written < length) {
        if (!bluetooth_.hasClient() ||
            static_cast<uint32_t>(millis() - startedAt) >= SEND_TIMEOUT_MS) {
            return false;
        }

        const size_t accepted = bluetooth_.write(
            reinterpret_cast<const uint8_t*>(payload) + written,
            length - written);
        if (accepted > 0) written += accepted;
        else vTaskDelay(pdMS_TO_TICKS(1));
    }

    const uint8_t newline = '\n';
    while (bluetooth_.hasClient() &&
           static_cast<uint32_t>(millis() - startedAt) < SEND_TIMEOUT_MS) {
        if (bluetooth_.write(&newline, 1) == 1) return true;
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    return false;
}

void TelemetryClient::run() {
    QueueItem item{};
    while (true) {
        const uint32_t now = millis();
        const bool wasConnected = connected_;
        connected_ = bluetooth_.hasClient();

        if (wasConnected && !connected_) {
            portENTER_CRITICAL(&stateMux_);
            if (sessionActive_ && !sessionClosedLocally_) sessionStartPending_ = true;
            portEXIT_CRITICAL(&stateMux_);
        }
        if (!wasConnected && connected_) snapshotReplayPending_ = true;

        if (!connected_) {
            // Drena amostras transitórias offline: mapa/configuração são mantidos
            // no cache, início e encerramento de sessão permanecem pendentes.
            portENTER_CRITICAL(&stateMux_);
            if (sessionActive_ && !sessionClosedLocally_) sessionStartPending_ = true;
            portEXIT_CRITICAL(&stateMux_);
            if (queue_ != nullptr && xQueueReceive(queue_, &item, 0) == pdTRUE) {
                ++droppedMessages_;
            }
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        // O backend deve conhecer a sessão antes de receber logs ou telemetria.
        bool startPending;
        char startPayload[kPayloadSize];
        size_t startLength;
        portENTER_CRITICAL(&stateMux_);
        startPending = sessionStartPending_;
        startLength = sessionStartLength_;
        if (startPending && startLength > 0) memcpy(startPayload, sessionStartPayload_, startLength + 1);
        portEXIT_CRITICAL(&stateMux_);
        if (startPending) {
            if (startLength > 0 && sendFrame(startPayload, startLength)) {
                portENTER_CRITICAL(&stateMux_);
                sessionStartPending_ = false;
                portEXIT_CRITICAL(&stateMux_);
            } else {
                connected_ = false;
            }
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        // Primeiro esvazia a fila corrente; só depois o snapshot corrige células
        // que não puderam ser transmitidas durante a desconexão.
        if (snapshotReplayPending_ && uxQueueMessagesWaiting(queue_) == 0) {
            if (replaySnapshot()) snapshotReplayPending_ = false;
            else connected_ = false;
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        // O fim respeita a ordem dos logs/telemetria já enfileirados antes dele.
        const SessionBuffer* endSession = nullptr;
        portENTER_CRITICAL(&stateMux_);
        if (pendingSessionEnd_ != nullptr && uxQueueMessagesWaiting(queue_) == 0 && !snapshotReplayPending_) {
            endSession = pendingSessionEnd_;
        }
        portEXIT_CRITICAL(&stateMux_);
        if (endSession != nullptr) {
            if (sendSession(*endSession)) {
                portENTER_CRITICAL(&stateMux_);
                if (pendingSessionEnd_ == endSession) pendingSessionEnd_ = nullptr;
                portEXIT_CRITICAL(&stateMux_);
            } else {
                connected_ = false;
            }
        }
        if (!connected_) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        if (static_cast<uint32_t>(now - lastQueueCheckMs_) >= 2 &&
            xQueueReceive(queue_, &item, 0) == pdTRUE) {
            lastQueueCheckMs_ = now;
            if (item.kind == ItemKind::Telemetry &&
                static_cast<uint32_t>(now - item.queuedAtMs) > network::TELEMETRY_QUEUE_STALE_MS) {
                ++droppedMessages_;
            } else if (!sendFrame(item.payload, strlen(item.payload))) {
                ++droppedMessages_;
                connected_ = false;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

bool TelemetryClient::replaySnapshot() {
    char payload[kPayloadSize];
    size_t length = 0;
    uint8_t width = 0;
    uint8_t height = 0;
    portENTER_CRITICAL(&stateMux_);
    length = mazeLength_;
    if (length > 0) memcpy(payload, mazePayload_, length + 1);
    width = mazeWidth_;
    height = mazeHeight_;
    portEXIT_CRITICAL(&stateMux_);
    if (length > 0 && !sendFrame(payload, length)) return false;

    for (uint8_t y = 0; y < height; ++y) {
        for (uint8_t x = 0; x < width; ++x) {
            const size_t index = static_cast<size_t>(y) * kMaxMazeWidth + x;
            portENTER_CRITICAL(&stateMux_);
            length = cellValid_[index] ? cellLengths_[index] : 0;
            if (length > 0) memcpy(payload, cellPayloads_[index], length + 1);
            portEXIT_CRITICAL(&stateMux_);
            if (length > 0 && !sendFrame(payload, length)) return false;
        }
    }

    // O último pacote define a pose mais recente depois de replayar células antigas.
    portENTER_CRITICAL(&stateMux_);
    length = latestCellLength_;
    if (length > 0) memcpy(payload, latestCellPayload_, length + 1);
    portEXIT_CRITICAL(&stateMux_);
    return length == 0 || sendFrame(payload, length);
}

bool TelemetryClient::sendSession(const SessionBuffer& session) {
    String body;
    body.reserve(256 + session.count() * 48);
    StringWriterContext writer{&body, true};
    if (!session.writeJson(appendSessionChunk, &writer) || !writer.ok || body.length() < 2 || body[0] != '{') {
        Serial.println("HU-11: falha ao serializar a sessão para Bluetooth SPP.");
        return false;
    }
    String envelope;
    envelope.reserve(body.length() + 24);
    envelope = "{\"type\":\"session_end\",";
    envelope.concat(body.c_str() + 1);
    return sendFrame(envelope.c_str(), envelope.length());
}

}  // namespace mm

#endif  // ARDUINO
