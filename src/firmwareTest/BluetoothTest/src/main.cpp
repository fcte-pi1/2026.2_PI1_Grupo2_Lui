#include <Arduino.h>
#include <BluetoothSerial.h>
#include "diagnostic_logger.hpp"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error "Bluetooth nao habilitado no core Arduino-ESP32 usado para este alvo."
#endif

#if !defined(CONFIG_BT_SPP_ENABLED)
#error "Bluetooth Classic SPP nao esta disponivel neste alvo. Use uma ESP32 com suporte a Bluetooth Classic."
#endif

namespace {
constexpr char BT_DEVICE_NAME[] = "MicroMouse_BT_Test";
constexpr uint32_t HEARTBEAT_INTERVAL_MS = 3000;
constexpr size_t MAX_RX_LINE_LENGTH = 120;

BluetoothSerial bluetooth;
String receivedLine;
bool bluetoothStarted = false;
bool wasConnected = false;
bool droppingLongLine = false;
uint32_t lastHeartbeatMs = 0;
uint32_t heartbeatSequence = 0;

void sendHeartbeat() {
    ++heartbeatSequence;
    bluetooth.print("PING ");
    bluetooth.print(heartbeatSequence);
    bluetooth.print(" | uptime_ms=");
    bluetooth.println(millis());
    lastHeartbeatMs = millis();
}

void handleIncomingData() {
    while (bluetooth.available() > 0) {
        const int value = bluetooth.read();
        if (value < 0) {
            break;
        }

        const char character = static_cast<char>(value);
        if (character == '\r') {
            continue;
        }

        if (character == '\n') {
            if (droppingLongLine) {
                bluetooth.println("ERRO: mensagem excedeu o limite de 120 caracteres.");
                DiagnosticLog::log("WARN", "Mensagem Bluetooth descartada: acima de %u caracteres.",
                                   static_cast<unsigned>(MAX_RX_LINE_LENGTH));
                droppingLongLine = false;
                receivedLine = "";
                continue;
            }

            if (receivedLine.length() == 0) {
                continue;
            }

            bluetooth.print("ECHO: ");
            bluetooth.println(receivedLine);
            DiagnosticLog::log("PASS", "Mensagem recebida via Bluetooth e ecoada: %s",
                               receivedLine.c_str());
            receivedLine = "";
            continue;
        }

        // Aceita caracteres imprimiveis e ignora controles inesperados.
        if (character >= 32 && character <= 126) {
            if (receivedLine.length() < MAX_RX_LINE_LENGTH) {
                receivedLine += character;
            } else {
                receivedLine = "";
                droppingLongLine = true;
            }
        }
    }
}
}  // namespace

void setup() {
    Serial.begin(115200);
    delay(400);
    Serial.setTimeout(100);

    DiagnosticLog::banner("Bluetooth Classic SPP — teste bidirecional");
    DiagnosticLog::log("INFO", "Inicializando Bluetooth Classic SPP; nome anunciado: %s", BT_DEVICE_NAME);

    bluetoothStarted = bluetooth.begin(BT_DEVICE_NAME);
    if (!bluetoothStarted) {
        DiagnosticLog::log("ERROR", "Falha ao inicializar o Bluetooth Classic. Verifique placa, core e suporte a SPP.");
        return;
    }

    DiagnosticLog::log("INFO", "Radio inicializado. Isso ainda NAO confirma conexao com outro dispositivo.");
    DiagnosticLog::log("ACTION", "No celular/computador, ative Bluetooth, procure %s e pareie/conecte.", BT_DEVICE_NAME);
    DiagnosticLog::log("ACTION", "Abra um terminal Bluetooth SPP, envie uma linha como teste e confira a resposta ECHO.");
    DiagnosticLog::log("INFO", "Baud do monitor USB: 115200. Nenhum pino externo e necessario para Bluetooth.");
}

void loop() {
    if (!bluetoothStarted) {
        delay(1000);
        return;
    }

    const bool connected = bluetooth.hasClient();
    if (connected && !wasConnected) {
        wasConnected = true;
        lastHeartbeatMs = millis();
        DiagnosticLog::log("PASS", "Conexao Bluetooth SPP estabelecida com um cliente.");
        bluetooth.println("MicroMouse Bluetooth Test conectado.");
        bluetooth.println("Envie uma mensagem terminada por ENTER; a ESP32 respondera ECHO.");
        bluetooth.println("Voce tambem recebera PING periodico.");
        sendHeartbeat();
    } else if (!connected && wasConnected) {
        wasConnected = false;
        receivedLine = "";
        droppingLongLine = false;
        DiagnosticLog::log("WARN", "Cliente Bluetooth desconectado; aguardando nova conexao.");
    }

    if (connected) {
        handleIncomingData();
        if (static_cast<uint32_t>(millis() - lastHeartbeatMs) >= HEARTBEAT_INTERVAL_MS) {
            sendHeartbeat();
        }
    }

    delay(5);
}
