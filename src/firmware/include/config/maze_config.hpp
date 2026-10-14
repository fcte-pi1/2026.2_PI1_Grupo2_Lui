#pragma once

#include <cstdint>

#include "common/robot_types.hpp"

enum class MazeFormat : uint8_t {
    Maze4x4,
    Maze8x4,
    Maze12x4
};

struct MazeConfig {
    MazeFormat format;

    uint8_t rows;
    uint8_t cols;

    CellPosition start;
    CellPosition goal;

    MazeCorner startCorner;
    Direction initialHeading;

    uint16_t cellSizeMm;

    uint32_t attemptTimeoutMs;
    uint8_t maxAttempts;
};

namespace MazeSettings {

    constexpr uint16_t CELL_SIZE_MM = 180;

    constexpr uint32_t ATTEMPT_TIMEOUT_MS =
        10UL * 60UL * 1000UL;

    constexpr uint8_t MAX_ATTEMPTS = 3;

}

inline CellPosition getCornerPosition(
    MazeCorner corner,
    uint8_t rows,
    uint8_t cols
) {
    switch (corner) {
        case MazeCorner::TopLeft:
            return {0, 0};

        case MazeCorner::TopRight:
            return {
                0,
                static_cast<uint8_t>(cols - 1)
            };

        case MazeCorner::BottomLeft:
            return {
                static_cast<uint8_t>(rows - 1),
                0
            };

        case MazeCorner::BottomRight:
            return {
                static_cast<uint8_t>(rows - 1),
                static_cast<uint8_t>(cols - 1)
            };
    }

    return {255, 255};
}

inline MazeCorner getOppositeCorner(MazeCorner corner) {
    switch (corner) {
        case MazeCorner::TopLeft:
            return MazeCorner::BottomRight;

        case MazeCorner::TopRight:
            return MazeCorner::BottomLeft;

        case MazeCorner::BottomLeft:
            return MazeCorner::TopRight;

        case MazeCorner::BottomRight:
            return MazeCorner::TopLeft;
    }

    return MazeCorner::TopLeft;
}


// Convenção de coordenadas:
// - row aumenta em direção ao sul;
// - col aumenta em direção ao leste;
// - a origem é a célula (0, 0);
// - o objetivo é a célula do canto oposto.

//                 COLUNAS
//              0   1   2   3
//            +---+---+---+---+
//    LINHA 0 | A |   |   | B |
//            +---+---+---+---+
//          1 |   |   |   |   |
//            +---+---+---+---+
//          2 |   |   |   |   |
//            +---+---+---+---+
//          3 | C |   |   | D |
//            +---+---+---+---+

inline MazeConfig makeMazeConfig(
    MazeFormat format,
    MazeCorner startCorner,
    Direction initialHeading
) {
    MazeConfig config{};

    config.format = format;
    config.cols = 4;

    switch (format) {
        case MazeFormat::Maze4x4:
            config.rows = 4;
            break;

        case MazeFormat::Maze8x4:
            config.rows = 8;
            break;

        case MazeFormat::Maze12x4:
            config.rows = 12;
            break;

        default:
            config.rows = 0;
            break;
    }

    config.startCorner = startCorner;
    config.initialHeading = initialHeading;

    config.cellSizeMm = MazeSettings::CELL_SIZE_MM;
    config.attemptTimeoutMs =
        MazeSettings::ATTEMPT_TIMEOUT_MS;
    config.maxAttempts = MazeSettings::MAX_ATTEMPTS;

    if (config.rows > 0) {
        config.start = getCornerPosition(
            startCorner,
            config.rows,
            config.cols
        );

        MazeCorner opposite = getOppositeCorner(
            startCorner
        );

        config.goal = getCornerPosition(
            opposite,
            config.rows,
            config.cols
        );
    }

    return config;
}

inline bool isValidMazeConfig(const MazeConfig& config) {
    if (config.cols != 4) {
        return false;
    }

    if (config.rows != 4 &&
        config.rows != 8 &&
        config.rows != 12) {
        return false;
    }

    if (config.start.row >= config.rows ||
        config.start.col >= config.cols) {
        return false;
    }

    if (config.goal.row != config.rows - 1 ||
        config.goal.col != config.cols - 1) {
        return false;
    }

    if (config.cellSizeMm == 0 ||
        config.attemptTimeoutMs == 0 ||
        config.maxAttempts == 0) {
        return false;
    }

    CellPosition expectedGoal = getCornerPosition(
        getOppositeCorner(config.startCorner),
        config.rows,
        config.cols
    );

    if (config.goal.row != expectedGoal.row ||
        config.goal.col != expectedGoal.col) {
        return false;
    }
    
    return true;
}