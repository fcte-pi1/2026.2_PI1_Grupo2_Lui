// TODO: descobrir quantas contagens correspondem a uma volta completa da roda.
/*
    1. Deixe o robô desligado e levante as rodas do chão.
    2. Marque uma posição de referência em uma das rodas.
    3. Zere a contagem do encoder.
    4. Gire a roda manualmente dez voltas completas.
    5. Registre a contagem final.
    6. Divida a quantidade de contagens por dez.
*/
// TODO: descobrir distancia entre o centro de cada roda
// diametro roda = 34mm

#pragma once

#include <stdint.h>

struct Pose2D
{
    float xMm;
    float yMm;
    float headingRad;
};

struct OdometryConfig
{
    float leftCountsPerMm;
    float rightCountsPerMm;
    float wheelTrackMm;
};

class Odometry
{
public:
    bool begin(const OdometryConfig &config,
               const Pose2D &initialPose,
               int32_t initialLeftCount,
               int32_t initialRightCount);

    bool update(int32_t currentLeftCount, int32_t currentRightCount);
    Pose2D getPose() const;
    float getLastDistanceMm() const;

    void setHeadingRad(float headingRad);

private:
    OdometryConfig config_{};
    Pose2D pose_{};
    int32_t previousLeftCount_ = 0;
    int32_t previousRightCount_ = 0;
    float lastDistanceMm_ = 0.0f;
    bool initialized_ = false;
};