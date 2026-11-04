# **Código C++ de Teste com Cálculo de Velocidade (RPM e cm/s)**

código C++ que calcula e exibe em tempo real a velocidade angular em **RPM (Rotações Por Minuto)** e a velocidade linear em **cm/s** para cada roda no Monitor Serial.

O cálculo é atualizado a cada **100 ms**, utilizando a variação instantânea dos pulsos dos encoders magnéticos (`GPIOs 32/33` e `25/26`) e a geometria das rodas N20 do seu robô.

```cpp
/*
 * ROBÔ MICROMOUSE - TESTE DE MOTORES COM VELOCIDADE REAL (RPM E CM/S)
 * Placa: ESP32 DevKit V1 (30 Pinos)
 * Driver: DRV8833
 * Motores: 2x N20 (6V) com Encoders Magnéticos
 */

#include <Arduino.h>

// ============================================================================
// 1. MAPEAMENTO DE PINOS (ESQUEMÁTICO DO MICROMOUSE)
// ============================================================================
// Ponte H DRV8833
#define MOTOR_L_PWM_PIN      16 // AIN1 (Velocidade/PWM Motor Esquerdo)
#define MOTOR_L_DIR_PIN      17 // AIN2 (Direção Motor Esquerdo)
#define MOTOR_R_PWM_PIN      18 // BIN1 (Velocidade/PWM Motor Direito)
#define MOTOR_R_DIR_PIN      19 // BIN2 (Direção Motor Direito)
#define DRV8833_EEP_PIN       5 // EEP / nSLEEP (Habilitação Lógica)

// Encoders Magnéticos
#define ENC_L_A_PIN          32 // Canal A Encoder Esquerdo
#define ENC_L_B_PIN          33 // Canal B Encoder Esquerdo
#define ENC_R_A_PIN          25 // Canal A Encoder Direito
#define ENC_R_B_PIN          26 // Canal B Encoder Direito

// ============================================================================
// 2. CONSTANTES FÍSICAS E CONFIGURAÇÃO DE VELOCIDADE
// ============================================================================
const float WHEEL_DIAMETER_CM = 3.2f;              // Diâmetro da roda N20 (3,2 cm)
const float WHEEL_CIRCUMFERENCE_CM = 10.053f;       // PI * Diâmetro (10,05 cm por volta)
const float PULSES_PER_REV = 206.0f;                // Pulsos por volta completa na saída da caixa
const float TICKS_PER_CM = 20.49f;                  // Pulsos por cm percorrido (PULSES_PER_REV / 10.053)

const unsigned long SPEED_CALC_INTERVAL_MS = 100;   // Intervalo de cálculo (100 ms = 10 Hz)

// Configuração PWM (LEDC)
const int PWM_FREQ = 20000;                         // 20 kHz (inaudível)
const int PWM_RESOLUTION = 8;                       // 8 bits (0 a 255)
const int PWM_CH_LEFT = 0;
const int PWM_CH_RIGHT = 1;

// ============================================================================
// 3. VARIÁVEIS GLOBAIS DE ENCODERS E TEMPO
// ============================================================================
volatile long leftEncoderTicks = 0;
volatile long rightEncoderTicks = 0;

long prevLeftTicks = 0;
long prevRightTicks = 0;
unsigned long lastSpeedCalcTime = 0;

float rpmLeft = 0.0f;
float rpmRight = 0.0f;
float speedCmSecLeft = 0.0f;
float speedCmSecRight = 0.0f;

// ============================================================================
// 4. INTERRUPÇÕES DE HARDWARE DOS ENCODERS
// ============================================================================
void IRAM_ATTR isrEncoderLeft() {
  if (digitalRead(ENC_L_B_PIN) == HIGH) leftEncoderTicks++;
  else leftEncoderTicks--;
}

void IRAM_ATTR isrEncoderRight() {
  if (digitalRead(ENC_R_B_PIN) == HIGH) rightEncoderTicks++;
  else rightEncoderTicks--;
}

// ============================================================================
// 5. FUNÇÕES DE CÁLCULO DE VELOCIDADE E MOTORES
// ============================================================================
void enableDRV8833(bool state) {
  digitalWrite(DRV8833_EEP_PIN, state ? HIGH : LOW);
}

void stopMotors() {
  digitalWrite(MOTOR_L_DIR_PIN, LOW);
  digitalWrite(MOTOR_R_DIR_PIN, LOW);
  ledcWrite(PWM_CH_LEFT, 0);
  ledcWrite(PWM_CH_RIGHT, 0);
}

void setMotorsSpeed(int leftSpeed, int rightSpeed) {
  // Motor Esquerdo
  if (leftSpeed >= 0) {
    digitalWrite(MOTOR_L_DIR_PIN, LOW);
    ledcWrite(PWM_CH_LEFT, constrain(leftSpeed, 0, 255));
  } else {
    digitalWrite(MOTOR_L_DIR_PIN, HIGH);
    ledcWrite(PWM_CH_LEFT, constrain(255 + leftSpeed, 0, 255));
  }

  // Motor Direito
  if (rightSpeed >= 0) {
    digitalWrite(MOTOR_R_DIR_PIN, LOW);
    ledcWrite(PWM_CH_RIGHT, constrain(rightSpeed, 0, 255));
  } else {
    digitalWrite(MOTOR_R_DIR_PIN, HIGH);
    ledcWrite(PWM_CH_RIGHT, constrain(255 + rightSpeed, 0, 255));
  }
}

// Calcula RPM e cm/s com base no intervalo decorrido
void updateSpeedMetrics() {
  unsigned long now = millis();
  unsigned long elapsedMs = now - lastSpeedCalcTime;

  if (elapsedMs >= SPEED_CALC_INTERVAL_MS) {
    // Leitura atômica dos ticks
    noInterrupts();
    long currentLeft = leftEncoderTicks;
    long currentRight = rightEncoderTicks;
    interrupts();

    // Variação de pulsos desde a última leitura
    long deltaLeft = currentLeft - prevLeftTicks;
    long deltaRight = currentRight - prevRightTicks;

    prevLeftTicks = currentLeft;
    prevRightTicks = currentRight;
    lastSpeedCalcTime = now;

    float elapsedSec = elapsedMs / 1000.0f;

    // Frequência de pulsos por segundo
    float ppsLeft = deltaLeft / elapsedSec;
    float ppsRight = deltaRight / elapsedSec;

    // Cálculo da Velocidade Angular (RPM)
    rpmLeft = (ppsLeft / PULSES_PER_REV) * 60.0f;
    rpmRight = (ppsRight / PULSES_PER_REV) * 60.0f;

    // Cálculo da Velocidade Linear (cm/s)
    speedCmSecLeft = (ppsLeft / TICKS_PER_CM);
    speedCmSecRight = (ppsRight / TICKS_PER_CM);

    // Exibição no Monitor Serial
    Serial.printf("[VELOCIDADE] Esq: %+6.1f RPM (%+5.1f cm/s) | Dir: %+6.1f RPM (%+5.1f cm/s) | Ticks: L=%ld, R=%ld\n",
                  rpmLeft, speedCmSecLeft, rpmRight, speedCmSecRight, currentLeft, currentRight);
  }
}

// Executa o passo de teste mantendo o cálculo contínuo de velocidade
void runTestStep(const char* stepName, int leftSpeed, int rightSpeed, int durationMs) {
  Serial.println("\n----------------------------------------------------------------------");
  Serial.println(stepName);
  Serial.println("----------------------------------------------------------------------");

  setMotorsSpeed(leftSpeed, rightSpeed);

  unsigned long startTime = millis();
  while (millis() - startTime < (unsigned long)durationMs) {
    updateSpeedMetrics();
    delay(10); // Pausa curta para não travar o loop
  }

  stopMotors();
  delay(300);
  updateSpeedMetrics(); // Leitura final após parada
}

// ============================================================================
// 6. SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=== TESTE DE MOTORES COM MEDIÇÃO REAL DE RPM E CM/S ===");

  pinMode(DRV8833_EEP_PIN, OUTPUT);
  pinMode(MOTOR_L_DIR_PIN, OUTPUT);
  pinMode(MOTOR_R_DIR_PIN, OUTPUT);
  enableDRV8833(false);

  pinMode(ENC_L_A_PIN, INPUT_PULLUP);
  pinMode(ENC_L_B_PIN, INPUT_PULLUP);
  pinMode(ENC_R_A_PIN, INPUT_PULLUP);
  pinMode(ENC_R_B_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_L_A_PIN), isrEncoderLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A_PIN), isrEncoderRight, RISING);

  ledcSetup(PWM_CH_LEFT, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(MOTOR_L_PWM_PIN, PWM_CH_LEFT);
  ledcSetup(PWM_CH_RIGHT, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(MOTOR_R_PWM_PIN, PWM_CH_RIGHT);

  stopMotors();
  lastSpeedCalcTime = millis();
  Serial.println("[OK] Sistema pronto. Iniciando em 3 segundos...");
  delay(3000);
}

// ============================================================================
// 7. LAÇO PRINCIPAL
// ============================================================================
void loop() {
  enableDRV8833(true);

  // 1. Velocidade Baixa (PWM 100)
  runTestStep("1. Avanço em Baixa Velocidade (PWM: 100)", 100, 100, 2500);

  // 2. Velocidade Média (PWM 160)
  runTestStep("2. Avanço em Média Velocidade (PWM: 160)", 160, 160, 2500);

  // 3. Velocidade Alta (PWM 220)
  runTestStep("3. Avanço em Alta Velocidade (PWM: 220)", 220, 220, 2500);

  // 4. Curva / Rotação Diferencial
  runTestStep("4. Rotação Diferencial (Motor Esq: -130, Motor Dir: +130)", -130, 130, 2000);

  Serial.println("\nCiclo de testes concluído. Pausa de 5 segundos...");
  enableDRV8833(false);
  delay(5000);
}
```

---

## **Como interpretar as medições no Monitor Serial:**
- **`RPM`**: Indica a velocidade de rotação da roda (ex: `+145.2 RPM`). Se uma das rodas apresentar valor negativo ao andar para frente, inverteu-se a sequência dos canais A/B dessa roda.
- **Comparação entre Rodas**: Se em PWM 160 o motor esquerdo marcar `22,0 cm/s` e o direito marcar `24,5 cm/s`, o código de controle PID do labirinto usará justamente esse sinal para compensar a diferença e manter a linha reta.
