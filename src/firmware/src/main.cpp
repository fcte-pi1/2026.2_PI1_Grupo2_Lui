#include <Arduino.h>

void setup() {
    Serial.begin(115200);

    Serial.println();
    Serial.println("================================");
    Serial.println("MicroMouse Firmware");
    Serial.println("ESP32 inicializado");
    Serial.println("================================");
}

void loop() {
    delay(1000);

    Serial.println("Firmware executando...");
}   