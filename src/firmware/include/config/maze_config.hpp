#pragma once

#include <cstdint>

#include "common/robot_types.hpp"
#include "config/maze_geometry.hpp"

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

    const bool formatMatches =
        (config.format == MazeFormat::Maze4x4 && config.rows == 4) ||
        (config.format == MazeFormat::Maze8x4 && config.rows == 8) ||
        (config.format == MazeFormat::Maze12x4 && config.rows == 12);
    if (!formatMatches) {
        return false;
    }

    if (config.start.row >= config.rows ||
        config.start.col >= config.cols ||
        config.goal.row >= config.rows ||
        config.goal.col >= config.cols) {
        return false;
    }

    if (config.cellSizeMm == 0 ||
        config.attemptTimeoutMs == 0 ||
        config.maxAttempts == 0) {
        return false;
    }

    if (static_cast<uint8_t>(config.startCorner) >
        static_cast<uint8_t>(MazeCorner::BottomRight) ||
        static_cast<uint8_t>(config.initialHeading) >
        static_cast<uint8_t>(Direction::West)) {
        return false;
    }

    const CellPosition expectedStart = getCornerPosition(
        config.startCorner, config.rows, config.cols);
    const CellPosition expectedGoal = getCornerPosition(
        getOppositeCorner(config.startCorner), config.rows, config.cols);

    return config.start.row == expectedStart.row &&
           config.start.col == expectedStart.col &&
           config.goal.row == expectedGoal.row &&
           config.goal.col == expectedGoal.col;
}
