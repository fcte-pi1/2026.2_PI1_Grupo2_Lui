#pragma once

/**
 * HU-00 - Construção contínua do mapa interno do labirinto.
 *
 * Convenção de coordenadas (alinhar com HU-02 e HU-08):
 *   - Origem (0,0) = célula de partida.
 *   - Norte = y + 1, Leste = x + 1.
 *   - Dir: North=0, East=1, South=2, West=3 (o valor é o bit na máscara).
 *
 * Duas máscaras por célula:
 *   walls -> bit = 1: há parede nesse lado
 *   known -> bit = 1: esse lado já foi medido por algum sensor
 * Sem `known` não dá para distinguir "sem parede" de "ainda não vi".
 *
 * Lógica pura: não depende de ESP-IDF/Arduino, roda em teste nativo.
 * Ocupa ~1,3 KB (16x16). Declare como variável global/estática.
 */

#include <cstddef>
#include <cstdint>

#include "hal/IMapStorage.h"

namespace mm {

enum class Dir : uint8_t { North = 0, East = 1, South = 2, West = 3 };

/** Direção relativa ao robô (somada ao heading vira direção absoluta). */
enum class Rel : uint8_t { Front = 0, Right = 1, Back = 2, Left = 3 };

namespace wall {
constexpr uint8_t N = 1u << static_cast<uint8_t>(Dir::North);
constexpr uint8_t E = 1u << static_cast<uint8_t>(Dir::East);
constexpr uint8_t S = 1u << static_cast<uint8_t>(Dir::South);
constexpr uint8_t W = 1u << static_cast<uint8_t>(Dir::West);
}  // namespace wall

struct Cell {
    uint8_t x;
    uint8_t y;
    uint8_t visited;   /* 0/1                              */
    uint8_t walls;     /* bitmask wall::*: parede presente */
    uint8_t known;     /* bitmask wall::*: lado já medido  */

    bool hasWall(Dir d) const { return (walls >> static_cast<uint8_t>(d)) & 1u; }
    bool isKnown(Dir d) const { return (known >> static_cast<uint8_t>(d)) & 1u; }
};
static_assert(sizeof(Cell) == 5, "Cell deve ter 5 bytes (formato do blob salvo)");

class MazeMap {
public:
    static constexpr uint8_t kMaxW = 16;
    static constexpr uint8_t kMaxH = 16;
    static constexpr size_t  kBlobHeader  = 8;
    static constexpr size_t  kBlobMaxSize = kBlobHeader + sizeof(Cell) * kMaxW * kMaxH + 4;

    MazeMap() : width_(0), height_(0), cells_{} {}

    /** Zera o mapa. Dimensões configuráveis até kMaxW x kMaxH (4x4, 8x4, 12x4, 16x16...). */
    bool init(uint8_t width, uint8_t height);

    uint8_t width() const { return width_; }
    uint8_t height() const { return height_; }

    static Dir relToAbs(Dir heading, Rel rel);
    static const char *dirStr(Dir d);

    /** nullptr se (x,y) estiver fora do labirinto. */
    const Cell *get(int x, int y) const;

    /** Marca/atualiza uma parede e a parede espelho na célula vizinha. */
    void setWall(int x, int y, Dir d, bool has_wall);

    /**
     * Chamar ao atingir o CENTRO de uma nova célula (CA-01 e CA-02).
     * Recebe as leituras binárias dos sensores (true = parede) - vindas da
     * HU-01 (WallPerception) -, converte para paredes absolutas usando o
     * heading (HU-02) e atualiza a célula.
     */
    bool updateCell(int x, int y, Dir heading,
                    bool wall_front, bool wall_left, bool wall_right);

    uint16_t visitedCount() const;

    /**
     * Pacote para o WebSocket (formato de células da HU-08):
     * {"robot":{"x":..,"y":..,"heading":"NORTH"},
     *  "cell":{"x":..,"y":..,"visited":true,
     *          "walls":{"north":true,"south":false,"east":true,"west":false}}}
     * Retorna bytes escritos ou -1 se não coube / célula inválida.
     */
    int cellPacketJson(int x, int y, Dir heading, char *buf, size_t size) const;

    /** Blob com magic, versão e CRC32. Retorna bytes ou 0. */
    size_t pack(uint8_t *out, size_t out_size) const;

    /** Valida e carrega o blob. false se corrompido ou de outra versão. */
    bool unpack(const uint8_t *in, size_t in_size);

    /** Persistência via HAL (NVS no robô, fake nos testes). */
    bool save(IMapStorage &storage) const;
    bool load(IMapStorage &storage);

private:
    bool inBounds(int x, int y) const {
        return x >= 0 && y >= 0 && x < width_ && y < height_;
    }

    uint8_t width_;
    uint8_t height_;
    Cell    cells_[kMaxH][kMaxW];   /* cells_[y][x] */
};

}  // namespace mm
