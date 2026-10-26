#include <Arduino.h>

// ============================================================================
// 1. MAPEAMENTO DE PINOS
// ============================================================================

// Motor esquerdo
#define MOTOR_L_PWM_PIN  16  // AIN1: PWM
#define MOTOR_L_DIR_PIN  17  // AIN2: direção

// Motor direito
#define MOTOR_R_PWM_PIN  18  // BIN1: PWM
#define MOTOR_R_DIR_PIN  19  // BIN2: direção

// Habilitação do driver
#define DRV8833_EEP_PIN   5  // nSLEEP: HIGH habilita; LOW desabilita

// ============================================================================
// 2. CONFIGURAÇÕES DO PWM
// ============================================================================

constexpr uint32_t PWM_FREQ = 20000;       // 20 kHz
constexpr uint8_t PWM_RESOLUTION = 8;      // Resolução de 8 bits
constexpr uint16_t PWM_MAX = 255;          // Duty cycle máximo

// ============================================================================
// 3. CONTROLE DA PONTE H
// ============================================================================

// Habilita ou desabilita o DRV8833.
void enableDRV8833(bool state) {
  digitalWrite(DRV8833_EEP_PIN, state ? HIGH : LOW);
}

// Para os dois motores por roda livre (coast).
void stopMotors() {
  digitalWrite(MOTOR_L_DIR_PIN, LOW);
  digitalWrite(MOTOR_R_DIR_PIN, LOW);

  ledcWrite(MOTOR_L_PWM_PIN, 0);
  ledcWrite(MOTOR_R_PWM_PIN, 0);
}

// Define a velocidade e o sentido dos motores.
//
// Valores aceitos: -255 a +255.
//
// Valor positivo: sentido direto.
// Valor negativo: sentido reverso.
// Valor zero: motor sem acionamento.
void setMotorsSpeed(int leftSpeed, int rightSpeed) {

  // Limita os valores recebidos ao intervalo permitido.
  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  // --------------------------------------------------------------------------
  // MOTOR ESQUERDO
  // --------------------------------------------------------------------------

  if (leftSpeed > 0) {
    // Sentido direto: AIN2 em LOW e PWM em AIN1.
    digitalWrite(MOTOR_L_DIR_PIN, LOW);
    ledcWrite(MOTOR_L_PWM_PIN, leftSpeed);

  } else if (leftSpeed < 0) {
    // Sentido reverso: AIN2 em HIGH.
    //
    // Como o PWM está aplicado a AIN1, o estado LOW de AIN1,
    // combinado com AIN2 em HIGH, aciona o sentido reverso.
    // O duty é invertido para ajustar a proporção de acionamento.
    digitalWrite(MOTOR_L_DIR_PIN, HIGH);
    ledcWrite(MOTOR_L_PWM_PIN, PWM_MAX + leftSpeed);

  } else {
    // Motor sem acionamento.
    digitalWrite(MOTOR_L_DIR_PIN, LOW);
    ledcWrite(MOTOR_L_PWM_PIN, 0);
  }

  // --------------------------------------------------------------------------
  // MOTOR DIREITO
  // --------------------------------------------------------------------------

  if (rightSpeed > 0) {
    // Sentido direto: BIN2 em LOW e PWM em BIN1.
    digitalWrite(MOTOR_R_DIR_PIN, LOW);
    ledcWrite(MOTOR_R_PWM_PIN, rightSpeed);

  } else if (rightSpeed < 0) {
    // Sentido reverso: BIN2 em HIGH.
    digitalWrite(MOTOR_R_DIR_PIN, HIGH);
    ledcWrite(MOTOR_R_PWM_PIN, PWM_MAX + rightSpeed);

  } else {
    // Motor sem acionamento.
    digitalWrite(MOTOR_R_DIR_PIN, LOW);
    ledcWrite(MOTOR_R_PWM_PIN, 0);
  }
}

// ============================================================================
// 4. CONFIGURAÇÃO
// ============================================================================

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("==============================================");
  Serial.println(" TESTE UNITARIO - MOTORES E PONTE H DRV8833");
  Serial.println("==============================================");

  // Configura os pinos de direção e habilitação.
  pinMode(DRV8833_EEP_PIN, OUTPUT);
  pinMode(MOTOR_L_DIR_PIN, OUTPUT);
  pinMode(MOTOR_R_DIR_PIN, OUTPUT);

  // Mantém o driver desabilitado durante a configuração.
  enableDRV8833(false);

  // --------------------------------------------------------------------------
  // Configuração do PWM para Arduino-ESP32 3.x
  // --------------------------------------------------------------------------

  // A função ledcAttach associa automaticamente um canal LEDC ao pino.
  bool pwmLeftOK = ledcAttach(
    MOTOR_L_PWM_PIN,
    PWM_FREQ,
    PWM_RESOLUTION
  );

  bool pwmRightOK = ledcAttach(
    MOTOR_R_PWM_PIN,
    PWM_FREQ,
    PWM_RESOLUTION
  );

  // Verifica se os dois sinais PWM foram configurados.
  if (!pwmLeftOK || !pwmRightOK) {
    Serial.println("[ERRO] Falha ao configurar o PWM.");

    stopMotors();
    enableDRV8833(false);

    while (true) {
      delay(1000);
    }
  }

  // Garante que os motores comecem sem acionamento.
  stopMotors();

  Serial.println("[OK] Pinos configurados.");
  Serial.println("[OK] PWM do motor esquerdo configurado.");
  Serial.println("[OK] PWM do motor direito configurado.");
  Serial.println("[OK] Frequencia PWM: 20000 Hz.");
  Serial.println("[OK] Resolucao PWM: 8 bits.");
  Serial.println();
  Serial.println("O teste iniciara em 3 segundos...");

  delay(3000);
}

// ============================================================================
// 5. ROTINA DE DIAGNÓSTICO
// ============================================================================

void loop() {

  // Habilita a Ponte H.
  enableDRV8833(true);

  // --------------------------------------------------------------------------
  // TESTE 1: MOVIMENTO PARA FRENTE
  // --------------------------------------------------------------------------

  Serial.println("1. Movendo para FRENTE (PWM: 120)...");

  setMotorsSpeed(120, 120);
  delay(2000);

  Serial.println("   Parando motores...");
  stopMotors();
  delay(1000);

  // --------------------------------------------------------------------------
  // TESTE 2: MOVIMENTO PARA TRÁS
  // --------------------------------------------------------------------------

  Serial.println("2. Movendo para TRAS (PWM: -120)...");

  setMotorsSpeed(-120, -120);
  delay(2000);

  stopMotors();
  delay(1000);

  // --------------------------------------------------------------------------
  // TESTE 3: GIRO PARA A ESQUERDA
  // --------------------------------------------------------------------------

  Serial.println(
    "3. Girando para ESQUERDA "
    "(Motor Esq: -100, Motor Dir: +100)..."
  );

  setMotorsSpeed(-100, 100);
  delay(1500);

  stopMotors();
  delay(1000);

  // --------------------------------------------------------------------------
  // TESTE 4: GIRO PARA A DIREITA
  // --------------------------------------------------------------------------

  Serial.println(
    "4. Girando para DIREITA "
    "(Motor Esq: +100, Motor Dir: -100)..."
  );

  setMotorsSpeed(100, -100);
  delay(1500);

  stopMotors();
  delay(1000);

  // --------------------------------------------------------------------------
  // TESTE 5: RAMPA DE ACELERAÇÃO
  // --------------------------------------------------------------------------

  Serial.println("5. Acelerando progressivamente de 0 a 255...");

  for (int speed = 0; speed <= PWM_MAX; speed += 15) {
    setMotorsSpeed(speed, speed);
    delay(100);
  }

  // Garante que o último valor também alcance 255.
  setMotorsSpeed(255, 255);
  delay(1000);

  // --------------------------------------------------------------------------
  // ENCERRAMENTO
  // --------------------------------------------------------------------------

  Serial.println(
    "Ciclo concluido. Parando motores e desabilitando o driver."
  );

  stopMotors();
  enableDRV8833(false);

  Serial.println("Driver desabilitado. Aguardando 5 segundos.");
  Serial.println();

  delay(5000);
}