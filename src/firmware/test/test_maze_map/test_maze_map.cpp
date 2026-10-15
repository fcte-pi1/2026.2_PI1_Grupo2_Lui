// Teste nativo da HU-00 (Unity).  pio test -e native -f test_maze_map
#include <unity.h>

#include <cstring>

#include "hal/IMapStorage.h"
#include "navigation/MazeMap.h"

using namespace mm;

namespace {

/* Fake em memória do HAL de armazenamento. */
class FakeStorage : public IMapStorage {
public:
    uint8_t data[MazeMap::kBlobMaxSize];
    size_t  len = 0;
    bool    fail_save = false;

    bool save(const uint8_t *d, size_t n) override {
        if (fail_save || n > sizeof(data)) return false;
        std::memcpy(data, d, n);
        len = n;
        return true;
    }
    size_t load(uint8_t *out, size_t max_len) override {
        if (len == 0 || len > max_len) return 0;
        std::memcpy(out, data, len);
        return len;
    }
};

MazeMap map;   // fora da stack

}  // namespace

void setUp(void) { map.init(8, 4); }
void tearDown(void) {}

void test_init_valida_dimensoes(void)
{
    TEST_ASSERT_FALSE(map.init(0, 4));
    TEST_ASSERT_FALSE(map.init(17, 4));
    TEST_ASSERT_TRUE(map.init(4, 4));
    TEST_ASSERT_TRUE(map.init(16, 16));
    TEST_ASSERT_EQUAL_UINT8(16, map.width());
    TEST_ASSERT_EQUAL_UINT8(0, map.get(5, 7)->visited);
    TEST_ASSERT_EQUAL_UINT8(5, map.get(5, 7)->x);
    TEST_ASSERT_EQUAL_UINT8(7, map.get(5, 7)->y);
}

void test_relativo_para_absoluto(void)
{
    TEST_ASSERT_EQUAL(Dir::East,  MazeMap::relToAbs(Dir::North, Rel::Right));
    TEST_ASSERT_EQUAL(Dir::West,  MazeMap::relToAbs(Dir::North, Rel::Left));
    TEST_ASSERT_EQUAL(Dir::East,  MazeMap::relToAbs(Dir::East,  Rel::Front));
    TEST_ASSERT_EQUAL(Dir::South, MazeMap::relToAbs(Dir::West,  Rel::Left));
    TEST_ASSERT_EQUAL(Dir::West,  MazeMap::relToAbs(Dir::South, Rel::Right));
    TEST_ASSERT_EQUAL(Dir::South, MazeMap::relToAbs(Dir::North, Rel::Back));
}

void test_update_cell_marca_visitada_e_paredes(void)
{
    // (0,0) olhando Norte: parede na frente (N) e à direita (L)
    TEST_ASSERT_TRUE(map.updateCell(0, 0, Dir::North, true, false, true));
    const Cell *c = map.get(0, 0);
    TEST_ASSERT_EQUAL_UINT8(1, c->visited);
    TEST_ASSERT_EQUAL_UINT8(wall::N | wall::E, c->walls);
    TEST_ASSERT_EQUAL_UINT8(wall::N | wall::E | wall::W, c->known);  // sul: desconhecido
    TEST_ASSERT_TRUE(c->hasWall(Dir::North));
    TEST_ASSERT_FALSE(c->hasWall(Dir::West));
    TEST_ASSERT_FALSE(c->isKnown(Dir::South));
}

void test_parede_espelho_na_vizinha(void)
{
    map.updateCell(0, 0, Dir::North, true, false, false);
    const Cell *up = map.get(0, 1);
    TEST_ASSERT_TRUE(up->hasWall(Dir::South));
    TEST_ASSERT_TRUE(up->isKnown(Dir::South));
    TEST_ASSERT_EQUAL_UINT8(0, up->visited);
}

void test_mesma_parede_com_headings_diferentes(void)
{
    map.updateCell(3, 2, Dir::East, true, false, false);   // frente = Leste
    TEST_ASSERT_EQUAL_UINT8(wall::E, map.get(3, 2)->walls);
    map.updateCell(5, 2, Dir::South, false, false, true);  // direita de quem olha Sul = Oeste
    TEST_ASSERT_EQUAL_UINT8(wall::W, map.get(5, 2)->walls);
}

void test_remedicao_corrige_parede(void)
{
    map.updateCell(1, 1, Dir::North, true, false, false);
    TEST_ASSERT_TRUE(map.get(1, 1)->hasWall(Dir::North));
    map.updateCell(1, 1, Dir::North, false, false, false); // leitura nova: sem parede
    TEST_ASSERT_FALSE(map.get(1, 1)->hasWall(Dir::North));
    TEST_ASSERT_FALSE(map.get(1, 2)->hasWall(Dir::South)); // espelho também
}

void test_fora_dos_limites(void)
{
    TEST_ASSERT_FALSE(map.updateCell(8, 0, Dir::North, false, false, false));
    TEST_ASSERT_FALSE(map.updateCell(0, 4, Dir::North, false, false, false));
    TEST_ASSERT_NULL(map.get(-1, 0));
    // parede na borda não estoura índice
    map.updateCell(0, 0, Dir::West, true, false, false);
    TEST_ASSERT_TRUE(map.get(0, 0)->hasWall(Dir::West));
}

void test_visited_count(void)
{
    map.updateCell(0, 0, Dir::North, false, false, false);
    map.updateCell(0, 1, Dir::North, false, false, false);
    map.updateCell(0, 1, Dir::North, false, false, false);   // repetida
    TEST_ASSERT_EQUAL_UINT16(2, map.visitedCount());
}

void test_pacote_json(void)
{
    map.updateCell(0, 0, Dir::North, true, false, true);
    char js[256];
    TEST_ASSERT_GREATER_THAN(0, map.cellPacketJson(0, 0, Dir::North, js, sizeof(js)));
    TEST_ASSERT_NOT_NULL(std::strstr(js, "\"heading\":\"NORTH\""));
    TEST_ASSERT_NOT_NULL(std::strstr(js, "\"visited\":true"));
    TEST_ASSERT_NOT_NULL(std::strstr(js, "\"north\":true"));
    TEST_ASSERT_NOT_NULL(std::strstr(js, "\"east\":true"));
    TEST_ASSERT_NOT_NULL(std::strstr(js, "\"west\":false"));
    TEST_ASSERT_EQUAL(-1, map.cellPacketJson(0, 0, Dir::North, js, 10));
    TEST_ASSERT_EQUAL(-1, map.cellPacketJson(9, 9, Dir::North, js, sizeof(js)));
}

void test_pack_unpack_e_crc(void)
{
    map.updateCell(0, 0, Dir::North, true, false, true);
    map.updateCell(3, 2, Dir::East, true, false, false);

    uint8_t blob[MazeMap::kBlobMaxSize];
    const size_t len = map.pack(blob, sizeof(blob));
    TEST_ASSERT_EQUAL(MazeMap::kBlobHeader + 8 * 4 * sizeof(Cell) + 4, len);

    static MazeMap other;
    TEST_ASSERT_TRUE(other.unpack(blob, len));
    TEST_ASSERT_EQUAL_UINT8(8, other.width());
    TEST_ASSERT_EQUAL_UINT8(4, other.height());
    TEST_ASSERT_EQUAL_UINT16(2, other.visitedCount());
    TEST_ASSERT_EQUAL_MEMORY(map.get(0, 0), other.get(0, 0), sizeof(Cell));

    blob[20] ^= 0x01;                               // corrompe 1 bit
    TEST_ASSERT_FALSE(other.unpack(blob, len));
    blob[20] ^= 0x01;
    TEST_ASSERT_FALSE(other.unpack(blob, len - 1)); // truncado
    blob[2] = 99;                                   // versão desconhecida
    TEST_ASSERT_FALSE(other.unpack(blob, len));
}

void test_save_load_via_storage(void)
{
    map.updateCell(2, 1, Dir::North, true, true, false);

    static FakeStorage st;
    static MazeMap restored;
    TEST_ASSERT_FALSE(restored.load(st));           // nada salvo ainda
    TEST_ASSERT_TRUE(map.save(st));
    TEST_ASSERT_TRUE(restored.load(st));
    TEST_ASSERT_EQUAL_UINT16(1, restored.visitedCount());
    TEST_ASSERT_EQUAL_UINT8(map.get(2, 1)->walls, restored.get(2, 1)->walls);

    st.fail_save = true;
    TEST_ASSERT_FALSE(map.save(st));                // falha do HAL propaga
}

int main(int, char **)
{
    UNITY_BEGIN();
    RUN_TEST(test_init_valida_dimensoes);
    RUN_TEST(test_relativo_para_absoluto);
    RUN_TEST(test_update_cell_marca_visitada_e_paredes);
    RUN_TEST(test_parede_espelho_na_vizinha);
    RUN_TEST(test_mesma_parede_com_headings_diferentes);
    RUN_TEST(test_remedicao_corrige_parede);
    RUN_TEST(test_fora_dos_limites);
    RUN_TEST(test_visited_count);
    RUN_TEST(test_pacote_json);
    RUN_TEST(test_pack_unpack_e_crc);
    RUN_TEST(test_save_load_via_storage);
    return UNITY_END();
}
