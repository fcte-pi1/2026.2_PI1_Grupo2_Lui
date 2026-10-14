#include <Arduino.h>

#include "config/mazeConfig.hpp"

#include <Wire.h>

#include "config/board_pins.hpp"

#include "hal/motor_drv8833.hpp"
#include "hal/encoder_n20.hpp"
#include "hal/imu_mpu6500.hpp"

#include "control/pid_controller.hpp"
#include "control/odometry.hpp"
#include "control/motion_controller.hpp"

MazeConfig mazeConfig;

// Drivers HAL
MotorDRV8833 motors;

EncoderN20 encoderLeft;
EncoderN20 encoderRight;

IMUMPU6500 imu;

// Controle e localização
Odometry odometry;

// TODO: Ajustar os parâmetros do PID para o robô real. Os valores atuais são apenas para teste.
PIDController leftPid(
    0.8f, 0.10f, 0.0f,
    -120.0f, 120.0f);

PIDController rightPid(
    0.8f, 0.10f, 0.0f,
    -120.0f, 120.0f);

// Controlador de movimento
MotionController motion(
    motors,
    encoderLeft,
    encoderRight,
    imu,
    odometry,
    leftPid,
    rightPid);

void setup()
{
    Serial.begin(115200);

    // TODO: Integrar com o dashboard para permitir a seleção do formato do labirinto e da orientação inicial do robô.
    // EAST só é correto se a saída da célula inicial
    mazeConfig = makeMazeConfig(
        MazeFormat::Maze4x4,
        MazeCorner::TopLeft,
        Direction::East);

    if (!isValidMazeConfig(mazeConfig))
    {
        Serial.println("Configuracao de labirinto invalida!");
        return;
    }

    Serial.printf(
        "Labirinto: %ux%u\n",
        mazeConfig.rows,
        mazeConfig.cols);

    Serial.printf(
        "Inicio: (%u, %u)\n",
        mazeConfig.start.row,
        mazeConfig.start.col);

    Serial.printf(
        "Objetivo: (%u, %u)\n",
        mazeConfig.goal.row,
        mazeConfig.goal.col);

    Wire.begin(
        BoardPins::I2C_SDA,
        BoardPins::I2C_SCL);

    Wire.setClock(BoardPins::I2C_CLOCK_HZ);

    const bool motorsOK = motors.begin(
        BoardPins::MOTOR_LEFT.in1,
        BoardPins::MOTOR_LEFT.in2,
        BoardPins::MOTOR_RIGHT.in1,
        BoardPins::MOTOR_RIGHT.in2,
        BoardPins::MOTOR_NSLEEP,
        BoardPins::MOTOR_PWM_HZ,
        BoardPins::MOTOR_PWM_BITS);

    const bool leftEncoderOK = encoderLeft.begin(
        BoardPins::ENCODER_LEFT_A,
        BoardPins::ENCODER_LEFT_B);

    const bool rightEncoderOK = encoderRight.begin(
        BoardPins::ENCODER_RIGHT_A,
        BoardPins::ENCODER_RIGHT_B);

    const bool imuOK = imu.begin(
        Wire,
        BoardPins::MPU6500_ADDRESS);

    // Impedir a execução se algum componente falhar.
    if (!motorsOK ||
        !leftEncoderOK ||
        !rightEncoderOK ||
        !imuOK)
    {

        Serial.println(
            "Falha na inicializacao do hardware.");

        motors.setEnabled(false);
        return;
    }

    Serial.println(
        "Calibrando IMU. Mantenha o robo parado.");

    if (!imu.calibrateGyroZ())
    {
        Serial.println(
            "Falha na calibracao da IMU.");

        motors.setEnabled(false);
        return;
    }

    MotionControllerConfig config;

    // Valores ilustrativos: substituir pelas calibrações reais.
    config.leftCountsPerMm = 6.74f;
    config.rightCountsPerMm = 6.74f;

    config.wheelTrackMm = 90.0f;
    config.cellSizeMm = 180.0f;

    config.imuYawSign = 1.0f;

    // TODO: Calibrar os parâmetros para o robô real. Os valores atuais são apenas para teste.
    config.leftFeedForwardPwmPerMmS = 0.8f;
    config.rightFeedForwardPwmPerMmS = 0.8f;

    config.leftStaticFrictionPwm = 20.0f;
    config.rightStaticFrictionPwm = 20.0f;

    // TODO: Calibrar os parâmetros para o robô real. Os valores atuais são apenas para teste.
    const Pose2D initialPose = {
        0.0f,
        0.0f,
        0.0f};

    if (!motion.begin(config, initialPose))
    {
        Serial.println(
            "Falha ao inicializar o MotionController.");

        motors.setEnabled(false);
        return;
    }

    motors.setEnabled(true);

    ready = true;

    Serial.println(
        "Controlador de movimento inicializado.");
}

void loop()
{
    // A navegação será integrada posteriormente.
}