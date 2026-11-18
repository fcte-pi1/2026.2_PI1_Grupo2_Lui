#include <Arduino.h>
#include <ArduinoJson.h>
#include <esp_random.h>
#include <BluetoothSerial.h>
#include <Wire.h>
#include <cmath>
#include <cstring>
#include "config/board_pins.hpp"
#include "hal/imu_mpu6500.hpp"
#include "relative_yaw.hpp"

namespace {
constexpr char DEVICE_NAME[] = "MicroMouse_BT_Test";
constexpr uint32_t PACKET_PERIOD_MS = 250;
BluetoothSerial bluetooth;
IMUMPU6500 imu;
MPU6500Reading reading{};
DiagnosticYaw diagnosticYaw;
uint32_t previousYawUs = 0;
bool radioReady = false, imuReady = false, imuValid = false;
bool streaming = true, wasConnected = false;
uint32_t sequence = 0, lastPacketMs = 0, lastImuMs = 0, lastStatusMs = 0;
uint32_t bootId=0;
char command[256]{};
size_t commandLength = 0;
bool droppingCommand = false;

void disableActuatorsAndSonar() {
    pinMode(BoardPins::MOTOR_NSLEEP, OUTPUT);
    digitalWrite(BoardPins::MOTOR_NSLEEP, LOW);
    const uint8_t pins[] = {BoardPins::MOTOR_LEFT.in1, BoardPins::MOTOR_LEFT.in2,
        BoardPins::MOTOR_RIGHT.in1, BoardPins::MOTOR_RIGHT.in2,
        BoardPins::ULTRASONIC_FRONT.trigger, BoardPins::ULTRASONIC_LEFT.trigger,
        BoardPins::ULTRASONIC_RIGHT.trigger};
    for (auto pin : pins) { pinMode(pin, OUTPUT); digitalWrite(pin, LOW); }
}
void numberOrNull(bool valid, float value, char* output, size_t length) {
    if (valid && std::isfinite(value)) snprintf(output, length, "%.4f", value);
    else snprintf(output, length, "null");
}
void sendInfo() {
    bluetooth.println("{\"type\":\"bt_test_info\",\"protocol\":1,\"source\":\"ESP32\",\"motors_enabled\":false,\"ultrasonic_enabled\":false}");
}
void sendTelemetry(uint32_t now) {
    char yaw[24], gyro[24], ax[24], ay[24], az[24];
    numberOrNull(imuValid, reading.yawDegrees, yaw, sizeof(yaw));
    numberOrNull(imuValid, reading.gyroZdps, gyro, sizeof(gyro));
    numberOrNull(imuValid, reading.accelXg, ax, sizeof(ax));
    numberOrNull(imuValid, reading.accelYg, ay, sizeof(ay));
    numberOrNull(imuValid, reading.accelZg, az, sizeof(az));
    char packet[512];
    const int length = snprintf(packet, sizeof(packet),
        "{\"type\":\"bt_telemetry_test\",\"seq\":%lu,\"uptime_ms\":%lu,\"heap_bytes\":%lu,"
        "\"motors_enabled\":false,\"ultrasonic_enabled\":false,"
        "\"imu\":{\"valid\":%s,\"yaw_deg\":%s,\"gyro_z_dps\":%s,\"accel_g\":{\"x\":%s,\"y\":%s,\"z\":%s}}}",
        static_cast<unsigned long>(++sequence), static_cast<unsigned long>(now),
        static_cast<unsigned long>(ESP.getFreeHeap()), imuValid ? "true" : "false",
        yaw, gyro, ax, ay, az);
    if (length > 0 && static_cast<size_t>(length) < sizeof(packet)) {
        bluetooth.write(reinterpret_cast<const uint8_t*>(packet), length);
        bluetooth.write('\n');
    }
}
void handleRemoteCommand() {
    JsonDocument doc;
    if(deserializeJson(doc,command) || doc["type"]!="robot_command" || !doc["id"].is<const char*>())return;
    const char* id=doc["id"];
    if(strlen(id)!=32)return;
    for(size_t i=0;i<32;++i)if(!isxdigit(static_cast<unsigned char>(id[i])))return;
    const bool stop=doc["action"]=="stop";
    if(stop)disableActuatorsAndSonar();
    JsonDocument ack;
    ack["type"]="robot_ack";ack["id"]=id;ack["accepted"]=stop;
    ack["message"]=stop?"Motores desabilitados (teste Bluetooth)":"Firmware de teste: nao executa locomocao";
    serializeJson(ack,bluetooth);bluetooth.println();
}
void sendRobotStatus() {
    bluetooth.printf("{\"type\":\"robot_status\",\"firmware\":\"diagnostic\",\"boot_id\":%lu,\"remote_enabled\":false,\"state\":\"ready\",\"width\":4,\"corner\":0,\"heading\":1}\n",static_cast<unsigned long>(bootId));
}
void handleBluetoothInput() {
    while (bluetooth.available()) {
        const char ch = static_cast<char>(bluetooth.read());
        if (ch == '\r') continue;
        if (ch == '\n') {
            command[commandLength] = 0;
            if (!droppingCommand && !strncmp(command, "PING ", 5)) {
                bool tokenOk = commandLength > 5;
                for (size_t i=5; i<commandLength; ++i) {
                    const char c=command[i];
                    tokenOk = tokenOk && ((c>='a'&&c<='z')||(c>='A'&&c<='Z')||
                               (c>='0'&&c<='9')||c=='-'||c=='_');
                }
                if (tokenOk) {
                    bluetooth.printf("{\"type\":\"pong\",\"token\":\"%s\",\"uptime_ms\":%lu}\n",
                        command+5, static_cast<unsigned long>(millis()));
                    Serial.println("PING recebido por Bluetooth; PONG enviado.");
                }
            }
            if(!droppingCommand && command[0]=='{')handleRemoteCommand();
            commandLength=0;droppingCommand=false;
        } else if (!droppingCommand) {
            if (commandLength+1 < sizeof(command)) command[commandLength++]=ch;
            else { commandLength=0;droppingCommand=true; }
        }
    }
}
} // namespace

void setup() {
    bootId=esp_random();if(!bootId)bootId=1;
    disableActuatorsAndSonar(); // before Serial, I2C, Bluetooth or any delay
    Serial.begin(115200);
    delay(200);
    Serial.println("BT TELEMETRIA PARADO: motores OFF; TRIGs LOW; nenhuma navegacao neste firmware.");
    Wire.begin(BoardPins::I2C_SDA, BoardPins::I2C_SCL);
    Wire.setClock(BoardPins::I2C_CLOCK_HZ);
    Wire.setTimeOut(20);
    imuReady=imu.begin(Wire, BoardPins::MPU6500_ADDRESS);
    radioReady=bluetooth.begin(DEVICE_NAME);
    if (imuReady) {
        Serial.println("Aquecimento 2s; calibracao 5s. Mantenha o robo imovel.");
        delay(2000); // Calibrate with the radio already on.
        imuReady=imu.calibrateGyroZ(1000, 5);
        imu.resetYaw();
        diagnosticYaw.reset();
        previousYawUs=micros();
    }
    Serial.printf("IMU %s; yaw com compensacao de repouso (teste USB).\n", imuReady?"OK":"indisponivel");
    Serial.printf("Bluetooth %s, nome=%s. USB115200. p/x=pausar envio; r=retomar; s=status.\n",
        radioReady?"iniciado":"FALHOU", DEVICE_NAME);
}

void loop() {
    while (Serial.available()) {
        const char c=Serial.read();
        if(c=='p'||c=='x') { streaming=false;Serial.println("Envio pausado; motores OFF."); }
        else if(c=='r') { streaming=true;Serial.println("Envio retomado; motores OFF."); }
        else if(c=='s'||c=='h') Serial.printf("BT=%s; stream=%s; quadros=%lu; motoresOFF; ultrassonsOFF.\n",
            radioReady&&bluetooth.hasClient()?"conectado":"aguardando",streaming?"ON":"OFF",static_cast<unsigned long>(sequence));
    }
    const uint32_t now=millis();
    if(imuReady && now-lastImuMs>=20) {
        lastImuMs=now;
        imuValid=imu.read(reading);
        if (imuValid) {
            const float dt=static_cast<float>(reading.timestampUs-previousYawUs)/1000000.0f;
            previousYawUs=reading.timestampUs;
            reading.yawDegrees=diagnosticYaw.update(reading.gyroZdps,
                reading.accelXg,reading.accelYg,reading.accelZg,dt);
        }
    }
    if(!radioReady){delay(10);return;}
    const bool connected=bluetooth.hasClient();
    if(connected&&!wasConnected){sendInfo();sendRobotStatus();Serial.println("Bluetooth SPP: cliente conectado.");}
    if(!connected&&wasConnected){commandLength=0;droppingCommand=false;Serial.println("Bluetooth SPP: cliente desconectado.");}
    wasConnected=connected;
    if(connected){
        handleBluetoothInput();
        if(streaming&&now-lastPacketMs>=PACKET_PERIOD_MS){lastPacketMs=now;sendTelemetry(now);sendRobotStatus();}
    }
    if(now-lastStatusMs>=5000){lastStatusMs=now;Serial.printf("BT=%s quadros=%lu motoresOFF sonarOFF\n",connected?"conectado":"aguardando",static_cast<unsigned long>(sequence));}
    delay(2);
}
