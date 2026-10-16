#include "navigation/MazeSolver.h"

namespace nav {

namespace {

/* N, E, S, W - mesma convenção do MazeMap: Norte diminui a linha (y). */
constexpr int8_t kDx[4] = { 0, 1, 0, -1 };
constexpr int8_t kDy[4] = { -1, 0, 1, 0 };

/* Ordem de preferência no empate, relativa ao heading. */
constexpr mm::Rel kPreference[4] = {
    mm::Rel::Front, mm::Rel::Right, mm::Rel::Left, mm::Rel::Back
};

mm::Dir opposite(mm::Dir d)
{
    return mm::MazeMap::relToAbs(d, mm::Rel::Back);
}

}  // namespace

bool MazeSolver::setGoal(uint8_t x, uint8_t y)
{
    if (map_.get(x, y) == nullptr) {
        return false;
    }
    goalX_   = x;
    goalY_   = y;
    hasGoal_ = true;
    return true;
}

bool MazeSolver::passable(int x, int y, mm::Dir d) const
{
    const int nx = x + kDx[static_cast<uint8_t>(d)];
    const int ny = y + kDy[static_cast<uint8_t>(d)];
    const mm::Cell *from = map_.get(x, y);
    const mm::Cell *to   = map_.get(nx, ny);
    if (from == nullptr || to == nullptr) {
        return false;   /* borda do labirinto */
    }
    /* Confere os dois lados: parede conhecida em qualquer um bloqueia. */
    if (from->isKnown(d) && from->hasWall(d)) {
        return false;
    }
    const mm::Dir back = opposite(d);
    if (to->isKnown(back) && to->hasWall(back)) {
        return false;
    }
    return true;
}

bool MazeSolver::recompute()
{
    if (!hasGoal_) {
        return false;
    }
    const uint8_t w = map_.width();
    const uint8_t h = map_.height();
    for (uint8_t y = 0; y < mm::MazeMap::kMaxH; y++) {
        for (uint8_t x = 0; x < mm::MazeMap::kMaxW; x++) {
            dist_[y][x] = kUnreachable;
        }
    }

    /* BFS a partir do objetivo. A fila nunca passa de w*h <= 256 entradas. */
    uint8_t queueX[mm::MazeMap::kMaxW * mm::MazeMap::kMaxH];
    uint8_t queueY[mm::MazeMap::kMaxW * mm::MazeMap::kMaxH];
    uint16_t head = 0;
    uint16_t tail = 0;

    dist_[goalY_][goalX_] = 0;
    queueX[tail] = goalX_;
    queueY[tail] = goalY_;
    tail++;

    while (head < tail) {
        const uint8_t cx = queueX[head];
        const uint8_t cy = queueY[head];
        head++;
        for (uint8_t i = 0; i < 4; i++) {
            const mm::Dir d = static_cast<mm::Dir>(i);
            const int nx = cx + kDx[i];
            const int ny = cy + kDy[i];
            if (nx < 0 || ny < 0 || nx >= w || ny >= h) {
                continue;
            }
            if (dist_[ny][nx] != kUnreachable || !passable(cx, cy, d)) {
                continue;
            }
            dist_[ny][nx] = static_cast<uint16_t>(dist_[cy][cx] + 1);
            queueX[tail] = static_cast<uint8_t>(nx);
            queueY[tail] = static_cast<uint8_t>(ny);
            tail++;
        }
    }
    return true;
}

uint16_t MazeSolver::distance(int x, int y) const
{
    if (map_.get(x, y) == nullptr) {
        return kUnreachable;
    }
    return dist_[y][x];
}

Decision MazeSolver::decide(int x, int y, mm::Dir heading, mm::Dir &out)
{
    if (!recompute() || map_.get(x, y) == nullptr) {
        return Decision::NoPath;
    }
    if (isGoal(x, y)) {
        return Decision::AtGoal;
    }
    const uint16_t here = dist_[y][x];
    if (here == kUnreachable) {
        return Decision::NoPath;
    }

    for (uint8_t i = 0; i < 4; i++) {
        const mm::Dir d = mm::MazeMap::relToAbs(heading, kPreference[i]);
        if (!passable(x, y, d)) {
            continue;
        }
        const int nx = x + kDx[static_cast<uint8_t>(d)];
        const int ny = y + kDy[static_cast<uint8_t>(d)];
        if (dist_[ny][nx] + 1 == here) {
            out = d;
            return Decision::Move;
        }
    }
    return Decision::NoPath;   /* inalcançável: não deveria ocorrer se here != kUnreachable */
}

}  // namespace nav
