#pragma once

#include "common/robot_types.hpp"

inline bool getNextCell(
    CellPosition current,
    Direction direction,
    uint8_t rows,
    uint8_t cols,
    CellPosition& next
) {
    next = current;

    switch (direction) {
        case Direction::North:
            if (current.row == 0) {
                return false;
            }

            next.row--;
            break;

        case Direction::East:
            if (current.col >= cols - 1) {
                return false;
            }

            next.col++;
            break;

        case Direction::South:
            if (current.row >= rows - 1) {
                return false;
            }

            next.row++;
            break;

        case Direction::West:
            if (current.col == 0) {
                return false;
            }

            next.col--;
            break;
    }

    return true;
}