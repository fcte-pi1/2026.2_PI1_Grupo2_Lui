#include "odometry.hpp"

#include <math.h>

namespace
{
    constexpr float PI = 3.14159265358979323846f;

    float normalizeAngle(float angle)
    {
        while (angle > PI)
            angle -= 2.0f * PI;
        while (angle < -PI)
            angle += 2.0f * PI;
        return angle;
    }
}

bool Odometry::begin(const OdometryConfig &config,
                     const Pose2D &initialPose,
                     int32_t initialLeftCount,
                     int32_t initialRightCount)
{

    if (config.leftCountsPerMm <= 0.0f ||
        config.rightCountsPerMm <= 0.0f ||
        config.wheelTrackMm <= 0.0f)
    {
        return false;
    }

    config_ = config;
    pose_ = initialPose;

    pose_.headingRad = normalizeAngle(pose_.headingRad);

    previousLeftCount_ = initialLeftCount;
    previousRightCount_ = initialRightCount;

    lastDistanceMm_ = 0.0f;
    initialized_ = true;

    return true;
}

bool Odometry::update(int32_t currentLeftCount, int32_t currentRightCount)
{
    if (!initialized_)
        return false;

    const int32_t deltaLeftCount = currentLeftCount - previousLeftCount_;
    const int32_t deltaRightCount = currentRightCount - previousRightCount_;
    previousLeftCount_ = currentLeftCount;
    previousRightCount_ = currentRightCount;

    const float leftDistance =
        static_cast<float>(deltaLeftCount) / config_.leftCountsPerMm;
    const float rightDistance =
        static_cast<float>(deltaRightCount) / config_.rightCountsPerMm;
    const float distance = (leftDistance + rightDistance) / 2.0f;
    const float deltaHeading =
        (rightDistance - leftDistance) / config_.wheelTrackMm;
    const float middleHeading = pose_.headingRad + deltaHeading / 2.0f;

    pose_.xMm += distance * cosf(middleHeading);
    pose_.yMm += distance * sinf(middleHeading);
    pose_.headingRad = normalizeAngle(pose_.headingRad + deltaHeading);
    lastDistanceMm_ = distance;
    return true;
}

Pose2D Odometry::getPose() const { return pose_; }
float Odometry::getLastDistanceMm() const { return lastDistanceMm_; }

void Odometry::setHeadingRad(float headingRad)
{
    pose_.headingRad = normalizeAngle(headingRad);
}