#pragma once

#include <cstdint>

enum class Direction : uint8_t {
    North,
    East,
    South,
    West
};

enum class WallState : uint8_t {
    Unknown,
    Open,
    Wall
};

enum class MazeCorner : uint8_t {
    TopLeft,
    TopRight,
    BottomLeft,
    BottomRight
};

struct CellPosition {
    uint8_t row;
    uint8_t col;
};

struct RobotPose {
    CellPosition cell;
    Direction heading;
};

enum class RunStatus : uint8_t {
    Idle,
    Exploring,
    Navigating,
    Completed,
    Failed,
    TimedOut
};