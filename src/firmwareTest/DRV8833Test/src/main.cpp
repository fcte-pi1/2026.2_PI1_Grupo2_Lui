#include <Arduino.h>
#include "diagnostic_logger.hpp"
#include "../../../firmware/include/config/board_pins.hpp"
#include "../../../firmware/src/hal/motor_drv8833.hpp"

namespace {
MotorDRV8833 motors;
constexpr int16_t CONNECTION_PWM = 42;  // ~16% de duty em resolucao de 8 bits
constexpr int16_t DIAGNOSTIC_PWM = 64;  // ~25% de duty em resolucao de 8 bits
constexpr uint32_t CONNECTION_PULSE_MS = 350;
constexpr uint32_t DIAGNOSTIC_PULSE_MS = 400;
bool initialized = false;

void stopAndDisable(const char* reason) {
    motors.stop();
    motors.setEnabled(false);
    DiagnosticLog::log("INFO", "Saida segura aplicada (%s): PWM=0, nSLEEP=LOW.", reason);
}

void pulseMotor(const char* name, bool left, int16_t speed, uint32_t durationMs, const char* phase) {
    DiagnosticLog::log("ACTION", "%s | motor %s | comando=%d/255 | duracao=%lu ms.",
        phase, name, static_cast<int>(speed), static_cast<unsigned long>(durationMs));
    if (left) motors.setLeftSpeed(speed); else motors.setRightSpeed(speed);
    delay(durationMs);
    motors.stop();
    delay(350);
    DiagnosticLog::log("INFO", "Motor %s parado entre acionamentos.", name);
}

void abortTest(const char* reason) {
    stopAndDisable(reason);
    DiagnosticLog::log("ERROR", "Diagnostico funcional BLOQUEADO: pre-verificacao de conexao nao foi aprovada.");
}
}  // namespace

void setup() {
    Serial.begin(115200);
    delay(300);
    Serial.setTimeout(30000);
    DiagnosticLog::banner("DRV8833 + 2 motores N20");
    DiagnosticLog::log("INFO", "Pinos: motor esquerdo IN1/IN2=GPIO%u/%u; direito=GPIO%u/%u; nSLEEP=GPIO%u.",
        BoardPins::MOTOR_LEFT.in1, BoardPins::MOTOR_LEFT.in2,
        BoardPins::MOTOR_RIGHT.in1, BoardPins::MOTOR_RIGHT.in2,
        BoardPins::MOTOR_NSLEEP);
    DiagnosticLog::log("INFO", "PWM configurado: %lu Hz, resolucao %u bits. O driver sera inicializado com nSLEEP em LOW.",
        static_cast<unsigned long>(BoardPins::MOTOR_PWM_HZ), BoardPins::MOTOR_PWM_BITS);

    initialized = motors.begin(BoardPins::MOTOR_LEFT.in1, BoardPins::MOTOR_LEFT.in2,
        BoardPins::MOTOR_RIGHT.in1, BoardPins::MOTOR_RIGHT.in2,
        BoardPins::MOTOR_NSLEEP, BoardPins::MOTOR_PWM_HZ, BoardPins::MOTOR_PWM_BITS);
    if (!initialized) {
        DiagnosticLog::log("ERROR", "Inicializacao/configuracao do driver falhou. Nenhum motor sera acionado.");
        return;
    }
    motors.stop();
    motors.setEnabled(false);
    DiagnosticLog::log("PASS", "Inicializacao local OK; nSLEEP=%s; PWM=0.", motors.isEnabled() ? "HIGH" : "LOW (seguro)");

    DiagnosticLog::log("STAGE", "FASE 1 - pre-verificacao de conexao do driver/motores.");
    DiagnosticLog::log("WARN", "O DRV8833 nao fornece retorno digital de presenca/falha ligado a ESP32. A conexao sera verificada por pulso de baixa potencia + confirmacao visual do operador.");
    if (!DiagnosticLog::answerYes("CONFIRME: bateria/fonte e GND comum estao corretos, chassi esta preso e AMBAS as rodas estao suspensas?")) {
        abortTest("operador nao confirmou as condicoes de seguranca");
        return;
    }

    DiagnosticLog::waitForEnter("Somente agora: habilitarei o driver e aplicarei pulsos curtos. Pressione ENTER para iniciar a PRE-VERIFICACAO.");
    motors.setEnabled(true);
    DiagnosticLog::log("INFO", "nSLEEP habilitado. Se houver ruido, travamento ou movimento inesperado, desligue a alimentacao imediatamente.");
    pulseMotor("ESQUERDO", true, CONNECTION_PWM, CONNECTION_PULSE_MS, "PRE-CHECK DE CONEXAO");
    pulseMotor("DIREITO", false, CONNECTION_PWM, CONNECTION_PULSE_MS, "PRE-CHECK DE CONEXAO");
    motors.setEnabled(false);

    if (!DiagnosticLog::answerYes("O teste preliminar confirmou que AMBOS os motores giraram durante os pulsos?")) {
        abortTest("movimento dos motores nao confirmado pelo operador");
        DiagnosticLog::log("ERROR", "Inspecione alimentacao dos motores, GND comum, pinos IN, nSLEEP, soldas, cabos e polaridade. Nao prossiga para o diagnostico funcional.");
        return;
    }
    DiagnosticLog::log("PASS", "PRE-CHECK de conexao aprovado manualmente para ponte DRV8833 e os dois canais de motor.");

    DiagnosticLog::log("STAGE", "FASE 2 - diagnostico funcional (direcao/frente-reverso) liberado apos conexao confirmada.");
    motors.setEnabled(true);
    pulseMotor("ESQUERDO", true, DIAGNOSTIC_PWM, DIAGNOSTIC_PULSE_MS, "DIAGNOSTICO SENTIDO A");
    pulseMotor("ESQUERDO", true, -DIAGNOSTIC_PWM, DIAGNOSTIC_PULSE_MS, "DIAGNOSTICO SENTIDO B");
    pulseMotor("DIREITO", false, DIAGNOSTIC_PWM, DIAGNOSTIC_PULSE_MS, "DIAGNOSTICO SENTIDO A");
    pulseMotor("DIREITO", false, -DIAGNOSTIC_PWM, DIAGNOSTIC_PULSE_MS, "DIAGNOSTICO SENTIDO B");
    stopAndDisable("fim do teste");
    DiagnosticLog::log("PASS", "Sequencia de comandos concluida. Confirme visualmente que os sentidos correspondem a frente/re; isso depende da montagem dos motores.");
    DiagnosticLog::log("INFO", "Nao foi possivel medir rpm/torque automaticamente: o circuito nao retorna feedback eletrico do driver/motor neste teste.");
}

void loop() {
    delay(1000);
}
