#pragma once

#ifdef ARDUINO

#include <cstddef>
#include <cstdint>
#include "communication/ILogTransport.h"
#include "navigation/SessionBuffer.h"

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <WebSocketsClient.h>

namespace mm {

/**
 * HU-10: transporte assíncrono entre ESP32 e gateway /ws.
 * Os produtores publicam em uma fila sem esperar pelo socket. Telemetria antiga
 * é descartada; configuração e células conhecidas ficam em cache e são
 * retransmitidas na reconexão. O início da sessão precede logs/telemetria.
 */
class TelemetryClient final : public ILogTransport {
public:
    TelemetryClient() = default;
    ~TelemetryClient() override = default;

    bool begin();
    bool publishJson(const char* json, size_t length);
    bool publishMazeConfig(const char* json, size_t length, uint8_t width, uint8_t height);
    bool publishMapSnapshot(uint8_t cellX, uint8_t cellY, const char* json, size_t length);
    bool publishSessionStart(const char* json, size_t length);
    bool publishTelemetry(const char* json, size_t length);
    bool publishSessionEnd(const SessionBuffer& session);
    bool sendLog(const char* payload, size_t length) override;
    bool connected() const { return connected_; }
    uint32_t droppedMessages() const { return droppedMessages_; }

private:
    static constexpr size_t kPayloadSize = 480;
    static constexpr uint8_t kMaxMazeWidth = 12;
    static constexpr uint8_t kMaxMazeHeight = 4;
    static constexpr size_t kMaxMapCells = kMaxMazeWidth * kMaxMazeHeight;

    enum class ItemKind : uint8_t { Json, Telemetry };
    struct QueueItem {
        ItemKind kind = ItemKind::Json;
        uint32_t queuedAtMs = 0;
        char payload[kPayloadSize] = {0};
    };

    static void taskEntry(void* context);
    static void socketEvent(WStype_t type, uint8_t* payload, size_t length);
    void run();
    bool enqueue(ItemKind kind, const char* json, size_t length);
    bool sendSession(const SessionBuffer& session);
    bool replaySnapshot();

    QueueHandle_t queue_ = nullptr;
    TaskHandle_t task_ = nullptr;
    WebSocketsClient webSocket_;
    volatile bool connected_ = false;
    volatile uint32_t droppedMessages_ = 0;
    bool socketConfigured_ = false;
    uint32_t lastWifiRetryMs_ = 0;
    uint32_t lastQueueCheckMs_ = 0;

    // Protege cache/session-start compartilhados entre loop de missão e task WS.
    portMUX_TYPE stateMux_ = portMUX_INITIALIZER_UNLOCKED;
    char mazePayload_[kPayloadSize] = {0};
    size_t mazeLength_ = 0;
    uint8_t mazeWidth_ = 0;
    uint8_t mazeHeight_ = 0;
    char cellPayloads_[kMaxMapCells][kPayloadSize] = {{0}};
    uint16_t cellLengths_[kMaxMapCells] = {0};
    bool cellValid_[kMaxMapCells] = {false};
    char latestCellPayload_[kPayloadSize] = {0};
    size_t latestCellLength_ = 0;
    bool snapshotReplayPending_ = false; // escrita/leitura principal na task WS

    char sessionStartPayload_[kPayloadSize] = {0};
    size_t sessionStartLength_ = 0;
    bool sessionStartPending_ = false;
    bool sessionActive_ = false;
    bool sessionClosedLocally_ = false;
    const SessionBuffer* pendingSessionEnd_ = nullptr;
};

}  // namespace mm

#endif  // ARDUINO
