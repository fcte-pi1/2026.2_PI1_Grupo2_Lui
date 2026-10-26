#pragma once

/**
 * HU-03 - Resolução autônoma do labirinto até o objetivo.
 *
 * Decide para qual lado o robô deve ir a partir do mapa interno (HU-00).
 * Algoritmo: flood fill (BFS a partir do objetivo) sobre o mapa conhecido.
 *
 *   - Lado com parede conhecida (known=1, walls=1) bloqueia a passagem.
 *   - Lado ainda não medido (known=0) conta como livre (otimista): o robô
 *     tenta o caminho mais curto possível e, ao descobrir uma parede nova,
 *     recalcula. É assim que a exploração converge até o objetivo.
 *   - Empate entre vizinhos é resolvido preferindo seguir em frente, depois
 *     direita, esquerda e, por último, dar ré (menos giros = mais rápido).
 *
 * Convenção de coordenadas: a mesma do mm::MazeMap e do PositionTracker
 * (x = coluna, y = linha, Norte diminui y, Leste aumenta x).
 *
 * Lógica pura: sem ESP-IDF/Arduino, sem alocação dinâmica, roda em teste
 * nativo. Ocupa ~0,5 KB de RAM (16x16). Declare como variável global/estática.
 *
 * Uso típico, a cada centro de célula, depois de MazeMap::updateCell():
 *     nav::MazeSolver solver(map);
 *     solver.setGoal(goalX, goalY);
 *     ...
 *     mm::Dir next;
 *     switch (solver.decide(x, y, heading, next)) {
 *     case nav::Decision::Move:   // girar para `next` e avançar uma célula
 *     case nav::Decision::AtGoal: // encerrar a tentativa
 *     case nav::Decision::NoPath: // sem caminho: tratar como falha
 *     }
 */

#include <cstdint>

#include "navigation/MazeMap.h"

namespace nav {

enum class Decision : uint8_t {
    Move,     /* `out` recebe a direção absoluta do próximo passo */
    AtGoal,   /* a célula atual é o objetivo                       */
    NoPath    /* objetivo inalcançável com as paredes conhecidas   */
};

class MazeSolver {
public:
    /** Distância devolvida para célula sem caminho até o objetivo. */
    static constexpr uint16_t kUnreachable = 0xFFFFu;

    /** `map` precisa viver mais que o solver (guardamos só a referência). */
    explicit MazeSolver(const mm::MazeMap &map)
        : map_(map), goalX_(0), goalY_(0), hasGoal_(false), dist_{} {}

    /** Define a célula objetivo. false se estiver fora do labirinto. */
    bool setGoal(uint8_t x, uint8_t y);

    /** Recalcula as distâncias até o objetivo. false se não há objetivo. */
    bool recompute();

    /** Passos até o objetivo; kUnreachable se sem caminho ou fora do mapa. */
    uint16_t distance(int x, int y) const;

    bool isGoal(int x, int y) const {
        return hasGoal_ && x == goalX_ && y == goalY_;
    }

    /**
     * Recalcula o flood fill com o mapa atual e escolhe o próximo passo a
     * partir de (x, y) com o robô virado para `heading`.
     */
    Decision decide(int x, int y, mm::Dir heading, mm::Dir &out);

private:
    /** Há passagem de (x, y) para o vizinho na direção d? */
    bool passable(int x, int y, mm::Dir d) const;

    const mm::MazeMap &map_;
    uint8_t  goalX_;
    uint8_t  goalY_;
    bool     hasGoal_;
    uint16_t dist_[mm::MazeMap::kMaxH][mm::MazeMap::kMaxW];   /* dist_[y][x] */
};

}  // namespace nav
