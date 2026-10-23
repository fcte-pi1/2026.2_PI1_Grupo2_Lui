#pragma once

#include <Arduino.h>
#include <esp_system.h>

namespace DiagnosticLog {

template <typename... Args>
inline void log(const char* level, const char* format, Args... args) {
    Serial.printf("[%10lu ms] [%-5s] ", static_cast<unsigned long>(millis()), level);
    Serial.printf(format, args...);
    Serial.println();
}

inline void banner(const char* component) {
    Serial.println();
    Serial.println("============================================================");
    log("INFO", "DIAGNOSTICO DE HARDWARE: %s", component);
    log("INFO", "Ordem obrigatoria: conexao primeiro; diagnostico depois.");
    log("INFO", "ESP32: %s | CPU: %u MHz | heap livre: %u bytes",
        ESP.getChipModel(), static_cast<unsigned>(ESP.getCpuFreqMHz()),
        static_cast<unsigned>(ESP.getFreeHeap()));
    log("INFO", "Motivo de reset ESP-IDF: %d", static_cast<int>(esp_reset_reason()));
    Serial.println("============================================================");
}

inline String readLine() {
    while (!Serial.available()) {
        delay(10);
    }
    String line = Serial.readStringUntil('\n');
    line.trim();
    return line;
}

inline void waitForEnter(const char* instruction) {
    while (Serial.available()) {
        (void)Serial.read();
    }
    log("ACTION", "%s", instruction);
    (void)readLine();
}

inline bool answerYes(const char* question) {
    log("ACTION", "%s Digite 's' para SIM ou 'n' para NAO.", question);
    const String answer = readLine();
    return answer.equalsIgnoreCase("s") || answer.equalsIgnoreCase("sim") ||
           answer.equalsIgnoreCase("y") || answer.equalsIgnoreCase("yes");
}

}  // namespace DiagnosticLog
