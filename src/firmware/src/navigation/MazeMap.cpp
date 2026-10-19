#include "navigation/MazeMap.h"

#include <cstdio>
#include <cstring>

namespace mm {

namespace {

/* N, E, S, W - mesma convenção do PositionTracker: Norte diminui a linha (y). */
constexpr int8_t kDx[4] = { 0, 1, 0, -1 };
constexpr int8_t kDy[4] = { -1, 0, 1, 0 };

constexpr uint8_t kMagic0  = 'M';
constexpr uint8_t kMagic1  = 'Z';
constexpr uint8_t kVersion = 1;

uint32_t crc32(const uint8_t *data, size_t len)
{
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            const uint32_t mask = (crc & 1u) ? 0xEDB88320u : 0u;
            crc = (crc >> 1) ^ mask;
        }
    }
    return ~crc;
}

void setSide(Cell &c, Dir d, bool has_wall)
{
    const uint8_t bit = static_cast<uint8_t>(1u << static_cast<uint8_t>(d));
    c.known = static_cast<uint8_t>(c.known | bit);
    if (has_wall) {
        c.walls = static_cast<uint8_t>(c.walls | bit);
    } else {
        c.walls = static_cast<uint8_t>(c.walls & ~bit);
    }
}

}  // namespace

bool MazeMap::init(uint8_t width, uint8_t height)
{
    if (width == 0 || height == 0 || width > kMaxW || height > kMaxH) {
        return false;
    }
    std::memset(cells_, 0, sizeof(cells_));
    width_  = width;
    height_ = height;
    for (uint8_t y = 0; y < height; y++) {
        for (uint8_t x = 0; x < width; x++) {
            cells_[y][x].x = x;
            cells_[y][x].y = y;
        }
    }
    return true;
}

Dir MazeMap::relToAbs(Dir heading, Rel rel)
{
    return static_cast<Dir>((static_cast<uint8_t>(heading) + static_cast<uint8_t>(rel)) & 0x3);
}

const char *MazeMap::dirStr(Dir d)
{
    switch (d) {
    case Dir::North: return "NORTH";
    case Dir::East:  return "EAST";
    case Dir::South: return "SOUTH";
    case Dir::West:  return "WEST";
    }
    return "NORTH";
}

const Cell *MazeMap::get(int x, int y) const
{
    return inBounds(x, y) ? &cells_[y][x] : nullptr;
}

void MazeMap::setWall(int x, int y, Dir d, bool has_wall)
{
    if (!inBounds(x, y)) {
        return;
    }
    setSide(cells_[y][x], d, has_wall);

    /* Parede espelho: o lado oposto da vizinha é a mesma parede física. */
    const int nx = x + kDx[static_cast<uint8_t>(d)];
    const int ny = y + kDy[static_cast<uint8_t>(d)];
    if (inBounds(nx, ny)) {
        setSide(cells_[ny][nx], static_cast<Dir>((static_cast<uint8_t>(d) + 2) & 0x3), has_wall);
    }
}

bool MazeMap::updateCell(int x, int y, Dir heading,
                         bool wall_front, bool wall_left, bool wall_right,
                         bool front_valid, bool left_valid, bool right_valid)
{
    if (!inBounds(x, y)) {
        return false;
    }
    cells_[y][x].visited = 1;
    if (front_valid) setWall(x, y, relToAbs(heading, Rel::Front), wall_front);
    if (left_valid)  setWall(x, y, relToAbs(heading, Rel::Left), wall_left);
    if (right_valid) setWall(x, y, relToAbs(heading, Rel::Right), wall_right);
    return true;
}

uint16_t MazeMap::visitedCount() const
{
    uint16_t n = 0;
    for (uint8_t y = 0; y < height_; y++) {
        for (uint8_t x = 0; x < width_; x++) {
            n = static_cast<uint16_t>(n + cells_[y][x].visited);
        }
    }
    return n;
}

int MazeMap::cellPacketJson(int x, int y, Dir heading, char *buf, size_t size) const
{
    const Cell *c = get(x, y);
    if (c == nullptr) {
        return -1;
    }
    auto b = [](bool v) { return v ? "true" : "false"; };
    const int n = std::snprintf(buf, size,
        "{\"robot\":{\"x\":%d,\"y\":%d,\"heading\":\"%s\"},"
        "\"cell\":{\"x\":%u,\"y\":%u,\"visited\":%s,"
        "\"walls\":{\"north\":%s,\"south\":%s,\"east\":%s,\"west\":%s}}}",
        x, y, dirStr(heading), c->x, c->y, b(c->visited != 0),
        b(c->hasWall(Dir::North)), b(c->hasWall(Dir::South)),
        b(c->hasWall(Dir::East)),  b(c->hasWall(Dir::West)));
    return (n < 0 || static_cast<size_t>(n) >= size) ? -1 : n;
}

size_t MazeMap::pack(uint8_t *out, size_t out_size) const
{
    const size_t total = kBlobHeader + static_cast<size_t>(width_) * height_ * sizeof(Cell) + 4;
    if (width_ == 0 || out_size < total) {
        return 0;
    }

    std::memset(out, 0, kBlobHeader);
    out[0] = kMagic0;
    out[1] = kMagic1;
    out[2] = kVersion;
    out[3] = width_;
    out[4] = height_;

    uint8_t *p = out + kBlobHeader;
    for (uint8_t y = 0; y < height_; y++) {
        for (uint8_t x = 0; x < width_; x++) {
            std::memcpy(p, &cells_[y][x], sizeof(Cell));
            p += sizeof(Cell);
        }
    }

    const uint32_t crc = crc32(out, static_cast<size_t>(p - out));
    p[0] = static_cast<uint8_t>(crc);
    p[1] = static_cast<uint8_t>(crc >> 8);
    p[2] = static_cast<uint8_t>(crc >> 16);
    p[3] = static_cast<uint8_t>(crc >> 24);
    return total;
}

bool MazeMap::unpack(const uint8_t *in, size_t in_size)
{
    if (in_size < kBlobHeader + 4 ||
        in[0] != kMagic0 || in[1] != kMagic1 || in[2] != kVersion) {
        return false;
    }
    const uint8_t w = in[3];
    const uint8_t h = in[4];
    if (w == 0 || h == 0 || w > kMaxW || h > kMaxH) {
        return false;
    }
    const size_t body = kBlobHeader + static_cast<size_t>(w) * h * sizeof(Cell);
    if (in_size != body + 4) {
        return false;
    }
    const uint32_t stored = static_cast<uint32_t>(in[body]) |
                            (static_cast<uint32_t>(in[body + 1]) << 8) |
                            (static_cast<uint32_t>(in[body + 2]) << 16) |
                            (static_cast<uint32_t>(in[body + 3]) << 24);
    if (crc32(in, body) != stored) {
        return false;
    }

    init(w, h);
    const uint8_t *p = in + kBlobHeader;
    for (uint8_t y = 0; y < h; y++) {
        for (uint8_t x = 0; x < w; x++) {
            std::memcpy(&cells_[y][x], p, sizeof(Cell));
            p += sizeof(Cell);
        }
    }
    return true;
}

bool MazeMap::save(IMapStorage &storage) const
{
    uint8_t blob[kBlobMaxSize];
    const size_t len = pack(blob, sizeof(blob));
    return len > 0 && storage.save(blob, len);
}

bool MazeMap::load(IMapStorage &storage)
{
    uint8_t blob[kBlobMaxSize];
    const size_t len = storage.load(blob, sizeof(blob));
    return len > 0 && unpack(blob, len);
}

}  // namespace mm
