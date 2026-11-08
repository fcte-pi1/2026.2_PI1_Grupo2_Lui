# CÓDIGO DE NAVEGAÇÃO E EXPLORAÇÃO DO LABIRINTO

O código implementa uma **Máquina de Estados Finos (FSM)** para exploração do labirinto (com base no avanço por células de **18 cm** e regra da parede/alinhamento), utilizando:
* **Filtro de Mediana** nos 3 sensores ultrassônicos (Frontal, Esquerdo, Direito);
* **Controle de Rotação Angular por IMU (MPU6500)** via I2C (`GPIO 21/22`) para garantir giros exatos de 90° e 180°;
* **Controle PID de Alinhamento Lateral e Odometria por Encoders** nos motores N20;
* **Driver Ponte H DRV8833** com pino de habilitação/emergência `EEP` (`GPIO 5`).

---

### **Código C++ Completo para Navegação no Labirinto**

```cpp
/*
 * ROBÔ MICROMOUSE - CÓDIGO DE NAVEGAÇÃO E EXPLORAÇÃO DO LABIRINTO
 * Placa: ESP32 DevKit V1 (30 Pinos)
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
const float CELL_SIZE_CM = 18.0f;       // Dimensão padrão da célula do labirinto (18 cm)
const float WALL_THRESHOLD_CM = 14.0f;   // Distância limite para detectar parede
const float TARGET_SIDE_DIST_CM = 6.0f;  // Distância ideal até a parede lateral (centralização)

// Resolução dos Encoders N20 (Pulsos por cm percorrido)
const float TICKS_PER_CM = 11.45f;       // ~206 pulsos para 18 cm de célula

// Configuração de PWM (LEDC)
const int PWM_FREQ = 20000;              // 20 kHz (inaudível aos motores)
const int PWM_RES = 8;                   // Resolução 8 bits (0-255)
const int PWM_CH_LEFT = 0;
const int PWM_CH_RIGHT = 1;
const int BASE_SPEED = 120;              // Velocidade base de avanço (0 a 255)

// Endereço I2C do Giroscópio MPU6500
#define MPU6500_ADDR 0x68

// ============================================================================
// 3. VARIÁVEIS GLOBAIS E MÁQUINA DE ESTADOS
// ============================================================================
enum RobotState {
  STATE_IDLE,          // Aguardando botão de start
  STATE_CHECK_WALLS,   // Lendo paredes na célula atual
  STATE_DECIDE_DIR,    // Algoritmo de escolha de direção (Mão Direita/Exploração)
  STATE_MOVE_FORWARD,  // Avançando 1 célula (18 cm) com alinhamento PID
  STATE_TURN_LEFT,     // Girando 90° à esquerda via IMU
  STATE_TURN_RIGHT,    // Girando 90° à direita via IMU
  STATE_TURN_180,      // Girando 180° (retorno em beco sem saída)
  STATE_STOP_EMERGENCY // Parada de segurança
};

RobotState currentState = STATE_IDLE;

// Odometria (Volátil para Interrupções)
volatile long leftTicks = 0;
volatile long rightTicks = 0;

// Variáveis de IMU
float gyroZOffset = 0.0f;
float currentYaw = 0.0f;
unsigned long lastGyroTime = 0;

// Variáveis dos Sensores de Distância
float distFront = 0.0f;
float distLeft  = 0.0f;
float distRight = 0.0f;

// Flags de Parede na Célula Atual
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
// 5. DRIVER DE MOTORES E PWM
// ============================================================================
void enableDRV8833(bool state) {
  digitalWrite(DRV8833_EEP_PIN, state ? HIGH : LOW);
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

void stopMotors() {
  digitalWrite(MOTOR_L_DIR_PIN, LOW);
  digitalWrite(MOTOR_R_DIR_PIN, LOW);
  ledcWrite(PWM_CH_LEFT, 0);
  ledcWrite(PWM_CH_RIGHT, 0);
}

// ============================================================================
// 6. LEITURA DE SENSORES ULTRASSÔNICOS COM FILTRO DE MEDIANA (3 AMOSTRAS)
// ============================================================================
float getDistance(int trigPin, int echoPin) {
  float samples;
  for (int i = 0; i < 3; i++) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    long duration = pulseIn(echoPin, HIGH, 20000); // Timeout 20ms
    if (duration == 0) samples[i] = 100.0f; // Sem obstáculo próximo
    else samples[i] = (duration * 0.0343f) / 2.0f;
    delayMicroseconds(200);
  }
  std::sort(samples, samples + 3);
  return samples; // Retorna a mediana
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
// 7. FUNÇÕES DE LEITURA DA IMU MPU6500 (EIXO Z / YAW)
// ============================================================================
void initMPU6500() {
  Wire.beginTransmission(MPU6500_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1
  Wire.write(0x00); // Wake up
  Wire.endTransmission(true);

  // Calibração de Offset do Giroscópio
  float sum = 0;
  for (int i = 0; i < 200; i++) {
    Wire.beginTransmission(MPU6500_ADDR);
    Wire.write(0x47); // GYRO_ZOUT_H
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
    float gzDps = (rawGz - gyroZOffset) / 131.0f; // Escala ±250 dps
    if (abs(gzDps) > 0.5f) { // Filtro de ruído morto
      currentYaw += gzDps * dt;
    }
  }
}

// ============================================================================
// 8. MANOBRAS DE ROTAÇÃO PRECISA CONTROLADAS POR IMU
// ============================================================================
void turnAngle(float targetAngleDeg) {
  stopMotors();
  delay(100);
  currentYaw = 0.0f;
  lastGyroTime = micros();

  int turnSpeed = 110;
  
  if (targetAngleDeg > 0) { // Giro à Esquerda (Yaw Positivo)
    setMotorsSpeed(-turnSpeed, turnSpeed);
    while (currentYaw < (targetAngleDeg - 1.5f)) {
      updateYaw();
      delay(2);
    }
  } else { // Giro à Direita (Yaw Negativo)
    setMotorsSpeed(turnSpeed, -turnSpeed);
    while (currentYaw > (targetAngleDeg + 1.5f)) {
      updateYaw();
      delay(2);
    }
  }
  stopMotors();
  delay(150); // Estabilização mecânica
}

// ============================================================================
// 9. AVANÇO DE 1 CÉLULA (18 CM) COM CONTROLE PID E ODOMETRIA
// ============================================================================
void moveOneCell() {
  leftTicks = 0;
  rightTicks = 0;
  currentYaw = 0.0f;
  lastGyroTime = micros();

  long targetTicks = (long)(CELL_SIZE_CM * TICKS_PER_CM);

  // Ganhos do PID de Alinhamento Lateral / Angular
  float Kp_side = 4.0f;
  float Kp_gyro = 2.5f;

  while ((abs(leftTicks) + abs(rightTicks)) / 2 < targetTicks) {
    updateYaw();
    readAllUltrasonics();

    // Parada de Emergência caso apareça parede frontal inesperada
    if (distFront < 5.0f) {
      stopMotors();
      tone(BUZZER_PIN, 1800, 300);
      break;
    }

    // Cálculo do Erro de Alinhamento
    float correction = 0.0f;

    if (wallLeft && wallRight) {
      // Centraliza entre as duas paredes laterais
      float error = distLeft - distRight;
      correction = error * Kp_side;
    } else if (wallLeft) {
      // Mantém distância constante da parede esquerda
      float error = distLeft - TARGET_SIDE_DIST_CM;
      correction = error * Kp_side;
    } else if (wallRight) {
      // Mantém distância constante da parede direita
      float error = TARGET_SIDE_DIST_CM - distRight;
      correction = error * Kp_side;
    } else {
      // Sem paredes laterais: usa o Giroscópio para manter linha reta
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
// 10. SETUP DO SISTEMA
// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== MICROMOUSE: ALGORITMO DE NAVEGAÇÃO NO LABIRINTO ===");

  // HMI
  pinMode(START_BTN_PIN, INPUT_PULLUP);
  pinMode(STATUS_LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);

  // Barramento I2C e IMU
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  initMPU6500();
  Serial.println("[OK] IMU MPU6500 Calibrada.");

  // Ultrassons
  pinMode(TRIG_FRONT_PIN, OUTPUT);
  pinMode(ECHO_FRONT_PIN, INPUT);
  pinMode(TRIG_LEFT_PIN, OUTPUT);
  pinMode(ECHO_LEFT_PIN, INPUT);
  pinMode(TRIG_RIGHT_PIN, OUTPUT);
  pinMode(ECHO_RIGHT_PIN, INPUT);

  // Ponte H DRV8833
  pinMode(DRV8833_EEP_PIN, OUTPUT);
  pinMode(MOTOR_L_DIR_PIN, OUTPUT);
  pinMode(MOTOR_R_DIR_PIN, OUTPUT);
  enableDRV8833(false);

  // PWM LEDC
  ledcSetup(PWM_CH_LEFT, PWM_FREQ, PWM_RES);
  ledcAttachPin(MOTOR_L_PWM_PIN, PWM_CH_LEFT);
  ledcSetup(PWM_CH_RIGHT, PWM_FREQ, PWM_RES);
  ledcAttachPin(MOTOR_R_PWM_PIN, PWM_CH_RIGHT);

  // Encoders
  pinMode(ENC_L_A_PIN, INPUT_PULLUP);
  pinMode(ENC_L_B_PIN, INPUT_PULLUP);
  pinMode(ENC_R_A_PIN, INPUT_PULLUP);
  pinMode(ENC_R_B_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_L_A_PIN), isrEncoderLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A_PIN), isrEncoderRight, RISING);

  // Bip de Inicialização Pronta
  tone(BUZZER_PIN, 2000, 150);
  digitalWrite(STATUS_LED_PIN, HIGH);
  Serial.println("Pressione o Botao de Start (GPIO 13) para iniciar a navegacao...");
}

// ============================================================================
// 11. LAÇO PRINCIPAL (MÁQUINA DE ESTADOS DO LABIRINTO)
// ============================================================================
void loop() {
  switch (currentState) {

    case STATE_IDLE:
      // Aguarda pressionar o botão de start no GPIO 13
      if (digitalRead(START_BTN_PIN) == LOW) {
        delay(50); // Debounce
        if (digitalRead(START_BTN_PIN) == LOW) {
          tone(BUZZER_PIN, 1500, 200);
          enableDRV8833(true);
          digitalWrite(STATUS_LED_PIN, HIGH);
          Serial.println("\n>>> INICIANDO EXPLORAÇÃO DO LABIRINTO! <<<");
          delay(1000);
          currentState = STATE_CHECK_WALLS;
        }
      }
      break;

    case STATE_CHECK_WALLS:
      // Leitura da célula atual
      readAllUltrasonics();
      Serial.printf("[CÉLULA] Paredes -> Freq: %d | Esq: %d | Dir: %d (F:%.1f, E:%.1f, D:%.1f)\n",
                    wallFront, wallLeft, wallRight, distFront, distLeft, distRight);
      currentState = STATE_DECIDE_DIR;
      break;

    case STATE_DECIDE_DIR:
      // Algoritmo da Regra da Mão Direita (Right-Hand Wall Follower)
      if (!wallRight) {
        Serial.println("-> Decisão: Girar a DIREITA e avançar");
        currentState = STATE_TURN_RIGHT;
      } else if (!wallFront) {
        Serial.println("-> Decisão: Avançar em FRENTE");
        currentState = STATE_MOVE_FORWARD;
      } else if (!wallLeft) {
        Serial.println("-> Decisão: Girar a ESQUERDA e avançar");
        currentState = STATE_TURN_LEFT;
      } else {
        Serial.println("-> Decisão: Beco sem saída! Girar 180°");
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
      Serial.println("!!! ROBÔ PARADO EM EMERGÊNCIA !!!");
      while (true) {
        tone(BUZZER_PIN, 1000, 100);
        delay(500);
      }
      break;
  }
}
```

---

### **Como o algoritmo funciona no labirinto:**
1. **Verificação da Célula (`STATE_CHECK_WALLS`):** Ao parar no centro de cada célula de 18 cm, o robô lê as distâncias com os 3 ultrassons e identifica a presença de paredes à **Frente**, **Esquerda** e **Direita**.
2. **Tomada de Decisão (`STATE_DECIDE_DIR`):** Utiliza a **Regra da Mão Direita** (prioriza seguir a parede da direita; se não houver parede à direita, gira 90° à direita e avança; se houver, tenta avançar; se não der, gira à esquerda; se for um beco sem saída, gira 180°).
3. **Avanço Preciso de Célula (`moveOneCell`):** Anda exatamente 18 cm contando os pulsos dos encoders N20. Enquanto avança, calcula o erro de centralização via ultrassons/IMU e ajusta dinamicamente a velocidade individual dos motores por PID.
4. **Giro Monitorado por IMU (`turnAngle`):** Os giros de +90°, -90° e 180° realizam a integração contínua do giroscópio MPU6500 no eixo Z, interrompendo os motores exatamente no ângulo correto.