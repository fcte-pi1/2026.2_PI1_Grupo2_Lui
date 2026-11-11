#include <Arduino.h>
/*
 * ROBÔ MICROMOUSE - CÓDIGO DE NAVEGAÇÃO E EXPLORAÇÃO DO LABIRINTO (CORRIGIDO)
 * Placa: ESP32 DevKit V1 (30 Pinos) - Compatível com ESP32 Core v2.x e v3.x
 * Sensores: 3x Ultrassônicos US-100 (Frontal, Esquerdo, Direito) + IMU MPU6500 (I2C)
 * Atuadores: Ponte H DRV8833 + 2x Motores N20 com Encoders Magnéticos
 */

#include <Wire.h>
#include <algorithm>

// ============================================================================
// 1. MAPEAMENTO DE PINOS (ESQUEMÁTICO DO MICROMOUSE)
// ============================================================================

// Barramento I2C (IMU MPU6500 + INA226)
#define I2C_SDA_PIN          21
#define I2C_SCL_PIN          22

// Sensores Ultrassônicos US-100 / HC-SR04
#define TRIG_FRONT_PIN        4 // Trigger Frontal
#define ECHO_FRONT_PIN       35 // Echo Frontal (GPI - Input Only)

#define TRIG_LEFT_PIN        14 // Trigger Esquerdo
#define ECHO_LEFT_PIN        36 // Echo Esquerdo (GPI - VP)

#define TRIG_RIGHT_PIN       27 // Trigger Direito
#define ECHO_RIGHT_PIN       39 // Echo Direito (GPI - VN)

// Driver de Motores - Ponte H DRV8833
#define MOTOR_L_PWM_PIN      16 // AIN1 (Velocidade Esquerda)
#define MOTOR_L_DIR_PIN      17 // AIN2 (Sentido Esquerda)
#define MOTOR_R_PWM_PIN      18 // BIN1 (Velocidade Direita)
#define MOTOR_R_DIR_PIN      19 // BIN2 (Sentido Direita)
#define DRV8833_EEP_PIN       5 // EEP / nSLEEP (Habilitação Lógica)

// Encoders Magnéticos dos Motores N20
#define ENC_L_A_PIN          32
#define ENC_L_B_PIN          33
#define ENC_R_A_PIN          25
#define ENC_R_B_PIN          26

// Interface Homem-Máquina (HMI)
#define START_BTN_PIN        13 // Botão de Start (com Pull-Up)
#define STATUS_LED_PIN        2 // LED Azul On-board
#define BUZZER_PIN           23 // Buzzer Passivo

// ============================================================================
// 2. CONSTANTES DE GEOMETRIA E PARÂMETROS DE CONTROLE
// ============================================================================
const float CELL_SIZE_CM = 18.0f;       // Dimensão da célula do labirinto (18 cm)
const float WALL_THRESHOLD_CM = 14.0f;   // Distância limite para detectar parede
const float TARGET_SIDE_DIST_CM = 6.0f;  // Distância ideal de centralização

const float TICKS_PER_CM = 11.45f;       // Resolução do encoder (~206 pulsos / 18 cm)

const int PWM_FREQ = 20000;              // 20 kHz (inaudível)
const int PWM_RES = 8;                   // Resolução 8 bits (0-255)
const int PWM_CH_LEFT = 0;
const int PWM_CH_RIGHT = 1;
const int BASE_SPEED = 120;              // Velocidade base

#define MPU6500_ADDR 0x68

// ============================================================================
// 3. MÁQUINA DE ESTADOS E VARIÁVEIS GLOBAIS
// ============================================================================
enum RobotState {
  STATE_IDLE,
  STATE_CHECK_WALLS,
  STATE_DECIDE_DIR,
  STATE_MOVE_FORWARD,
  STATE_TURN_LEFT,
  STATE_TURN_RIGHT,
  STATE_TURN_180,
  STATE_STOP_EMERGENCY
};

RobotState currentState = STATE_IDLE;

volatile long leftTicks = 0;
volatile long rightTicks = 0;

float gyroZOffset = 0.0f;
float currentYaw = 0.0f;
unsigned long lastGyroTime = 0;

float distFront = 0.0f;
float distLeft  = 0.0f;
float distRight = 0.0f;

bool wallFront = false;
bool wallLeft  = false;
bool wallRight = false;

// ============================================================================
// 4. INTERRUPÇÕES DOS ENCODERS
// ============================================================================
void IRAM_ATTR isrEncoderLeft() {
  if (digitalRead(ENC_L_B_PIN) == HIGH) leftTicks++;
  else leftTicks--;
}

void IRAM_ATTR isrEncoderRight() {
  if (digitalRead(ENC_R_B_PIN) == HIGH) rightTicks++;
  else rightTicks--;
}

// ============================================================================
// 5. ABSTRAÇÃO DE CONTROLE PWM (COMPATÍVEL COM ESP32 CORE v2.x E v3.x)
// ============================================================================
void initPWM() {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(MOTOR_L_PWM_PIN, PWM_FREQ, PWM_RES);
  ledcAttach(MOTOR_R_PWM_PIN, PWM_FREQ, PWM_RES);
#else
  ledcSetup(PWM_CH_LEFT, PWM_FREQ, PWM_RES);
  ledcAttachPin(MOTOR_L_PWM_PIN, PWM_CH_LEFT);
  ledcSetup(PWM_CH_RIGHT, PWM_FREQ, PWM_RES);
  ledcAttachPin(MOTOR_R_PWM_PIN, PWM_CH_RIGHT);
#endif
}

void writePWM(int pin, int channel, int duty) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(pin, duty);
#else
  ledcWrite(channel, duty);
#endif
}

void enableDRV8833(bool state) {
  digitalWrite(DRV8833_EEP_PIN, state ? HIGH : LOW);
}

void setMotorsSpeed(int leftSpeed, int rightSpeed) {
  // Motor Esquerdo
  if (leftSpeed >= 0) {
    digitalWrite(MOTOR_L_DIR_PIN, LOW);
    writePWM(MOTOR_L_PWM_PIN, PWM_CH_LEFT, constrain(leftSpeed, 0, 255));
  } else {
    digitalWrite(MOTOR_L_DIR_PIN, HIGH);
    writePWM(MOTOR_L_PWM_PIN, PWM_CH_LEFT, constrain(255 + leftSpeed, 0, 255));
  }

  // Motor Direito
  if (rightSpeed >= 0) {
    digitalWrite(MOTOR_R_DIR_PIN, LOW);
    writePWM(MOTOR_R_PWM_PIN, PWM_CH_RIGHT, constrain(rightSpeed, 0, 255));
  } else {
    digitalWrite(MOTOR_R_DIR_PIN, HIGH);
    writePWM(MOTOR_R_PWM_PIN, PWM_CH_RIGHT, constrain(255 + rightSpeed, 0, 255));
  }
}

void stopMotors() {
  digitalWrite(MOTOR_L_DIR_PIN, LOW);
  digitalWrite(MOTOR_R_DIR_PIN, LOW);
  writePWM(MOTOR_L_PWM_PIN, PWM_CH_LEFT, 0);
  writePWM(MOTOR_R_PWM_PIN, PWM_CH_RIGHT, 0);
}

// ============================================================================
// 6. FILTRO DE MEDIANA DOS ULTRASSÔNICOS (CORRIGIDO)
// ============================================================================
float getDistance(int trigPin, int echoPin) {
  float samples[3]; // Vetor para armazenar 3 leituras do sensor

  for (int i = 0; i < 3; i++) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    long duration = pulseIn(echoPin, HIGH, 20000); // Timeout 20ms
    if (duration == 0) samples[i] = 100.0f;
    else samples[i] = (duration * 0.0343f) / 2.0f;
    delayMicroseconds(200);
  }

  std::sort(samples, samples + 3); // Ordena os 3 elementos
  return samples[1]; // Retorna a mediana das 3 leituras
}

void readAllUltrasonics() {
  distFront = getDistance(TRIG_FRONT_PIN, ECHO_FRONT_PIN);
  distLeft  = getDistance(TRIG_LEFT_PIN, ECHO_LEFT_PIN);
  distRight = getDistance(TRIG_RIGHT_PIN, ECHO_RIGHT_PIN);

  wallFront = (distFront < WALL_THRESHOLD_CM);
  wallLeft  = (distLeft  < WALL_THRESHOLD_CM);
  wallRight = (distRight < WALL_THRESHOLD_CM);
}

// ============================================================================
// 7. MPU6500 (GIROSCÓPIO / YAW)
// ============================================================================
void initMPU6500() {
  Wire.beginTransmission(MPU6500_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);

  float sum = 0;
  for (int i = 0; i < 200; i++) {
    Wire.beginTransmission(MPU6500_ADDR);
    Wire.write(0x47);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6500_ADDR, 2, true);
    int16_t rawGz = (Wire.read() << 8) | Wire.read();
    sum += rawGz;
    delay(5);
  }
  gyroZOffset = sum / 200.0f;
  lastGyroTime = micros();
}

void updateYaw() {
  unsigned long now = micros();
  float dt = (now - lastGyroTime) / 1000000.0f;
  lastGyroTime = now;

  Wire.beginTransmission(MPU6500_ADDR);
  Wire.write(0x47);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6500_ADDR, 2, true);
  if (Wire.available() >= 2) {
    int16_t rawGz = (Wire.read() << 8) | Wire.read();
    float gzDps = (rawGz - gyroZOffset) / 131.0f;
    if (abs(gzDps) > 0.5f) {
      currentYaw += gzDps * dt;
    }
  }
}

// ============================================================================
// 8. ROTAÇÃO IMU
// ============================================================================
void turnAngle(float targetAngleDeg) {
  stopMotors();
  delay(100);
  currentYaw = 0.0f;
  lastGyroTime = micros();

  int turnSpeed = 110;

  if (targetAngleDeg > 0) { // Giro à Esquerda
    setMotorsSpeed(-turnSpeed, turnSpeed);
    while (currentYaw < (targetAngleDeg - 1.5f)) {
      updateYaw();
      delay(2);
    }
  } else { // Giro à Direita
    setMotorsSpeed(turnSpeed, -turnSpeed);
    while (currentYaw > (targetAngleDeg + 1.5f)) {
      updateYaw();
      delay(2);
    }
  }
  stopMotors();
  delay(150);
}

// ============================================================================
// 9. AVANÇO DE CÉLULA (18 CM)
// ============================================================================
void moveOneCell() {
  leftTicks = 0;
  rightTicks = 0;
  currentYaw = 0.0f;
  lastGyroTime = micros();

  long targetTicks = (long)(CELL_SIZE_CM * TICKS_PER_CM);

  float Kp_side = 4.0f;
  float Kp_gyro = 2.5f;

  while ((abs(leftTicks) + abs(rightTicks)) / 2 < targetTicks) {
    updateYaw();
    readAllUltrasonics();

    if (distFront < 5.0f) {
      stopMotors();
      tone(BUZZER_PIN, 1800, 300);
      break;
    }

    float correction = 0.0f;

    if (wallLeft && wallRight) {
      float error = distLeft - distRight;
      correction = error * Kp_side;
    } else if (wallLeft) {
      float error = distLeft - TARGET_SIDE_DIST_CM;
      correction = error * Kp_side;
    } else if (wallRight) {
      float error = TARGET_SIDE_DIST_CM - distRight;
      correction = error * Kp_side;
    } else {
      correction = -currentYaw * Kp_gyro;
    }

    int leftSpeed  = constrain(BASE_SPEED - correction, 60, 200);
    int rightSpeed = constrain(BASE_SPEED + correction, 60, 200);

    setMotorsSpeed(leftSpeed, rightSpeed);
    delay(10);
  }

  stopMotors();
  delay(100);
}

// ============================================================================
// 10. SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== MICROMOUSE: ALGORITMO DE NAVEGAÇÃO ===");

  pinMode(START_BTN_PIN, INPUT_PULLUP);
  pinMode(STATUS_LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  initMPU6500();
  Serial.println("[OK] IMU MPU6500 Calibrada.");

  pinMode(TRIG_FRONT_PIN, OUTPUT);
  pinMode(ECHO_FRONT_PIN, INPUT);
  pinMode(TRIG_LEFT_PIN, OUTPUT);
  pinMode(ECHO_LEFT_PIN, INPUT);
  pinMode(TRIG_RIGHT_PIN, OUTPUT);
  pinMode(ECHO_RIGHT_PIN, INPUT);

  pinMode(DRV8833_EEP_PIN, OUTPUT);
  pinMode(MOTOR_L_DIR_PIN, OUTPUT);
  pinMode(MOTOR_R_DIR_PIN, OUTPUT);
  enableDRV8833(false);

  // Inicialização do PWM compatível com v2.x e v3.x
  initPWM();

  pinMode(ENC_L_A_PIN, INPUT_PULLUP);
  pinMode(ENC_L_B_PIN, INPUT_PULLUP);
  pinMode(ENC_R_A_PIN, INPUT_PULLUP);
  pinMode(ENC_R_B_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_L_A_PIN), isrEncoderLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A_PIN), isrEncoderRight, RISING);

  tone(BUZZER_PIN, 2000, 150);
  digitalWrite(STATUS_LED_PIN, HIGH);
  Serial.println("Pressione o Botao de Start (GPIO 13) para iniciar...");
}

// ============================================================================
// 11. LOOP PRINCIPAL
// ============================================================================
void loop() {
  switch (currentState) {

    case STATE_IDLE:
      if (digitalRead(START_BTN_PIN) == LOW) {
        delay(50);
        if (digitalRead(START_BTN_PIN) == LOW) {
          tone(BUZZER_PIN, 1500, 200);
          enableDRV8833(true);
          digitalWrite(STATUS_LED_PIN, HIGH);
          Serial.println("\n>>> INICIANDO EXPLORAÇÃO! <<<");
          delay(1000);
          currentState = STATE_CHECK_WALLS;
        }
      }
      break;

    case STATE_CHECK_WALLS:
      readAllUltrasonics();
      Serial.printf("[CÉLULA] Freq: %d | Esq: %d | Dir: %d (F:%.1f, E:%.1f, D:%.1f)\n",
                    wallFront, wallLeft, wallRight, distFront, distLeft, distRight);
      currentState = STATE_DECIDE_DIR;
      break;

    case STATE_DECIDE_DIR:
      if (!wallRight) {
        currentState = STATE_TURN_RIGHT;
      } else if (!wallFront) {
        currentState = STATE_MOVE_FORWARD;
      } else if (!wallLeft) {
        currentState = STATE_TURN_LEFT;
      } else {
        currentState = STATE_TURN_180;
      }
      break;

    case STATE_MOVE_FORWARD:
      moveOneCell();
      currentState = STATE_CHECK_WALLS;
      break;

    case STATE_TURN_RIGHT:
      turnAngle(-90.0f);
      moveOneCell();
      currentState = STATE_CHECK_WALLS;
      break;

    case STATE_TURN_LEFT:
      turnAngle(90.0f);
      moveOneCell();
      currentState = STATE_CHECK_WALLS;
      break;

    case STATE_TURN_180:
      turnAngle(180.0f);
      moveOneCell();
      currentState = STATE_CHECK_WALLS;
      break;

    case STATE_STOP_EMERGENCY:
      stopMotors();
      enableDRV8833(false);
      digitalWrite(STATUS_LED_PIN, LOW);
      while (true) {
        tone(BUZZER_PIN, 1000, 100);
        delay(500);
      }
      break;
  }
}