#pragma once

/**
 * HU-11 - Armazenamento das informações de navegação.
 *
 * Buffer em RAM com a rota percorrida (uma entrada por célula). Nada é gravado
 * em disco durante o percurso (sem gargalo de I/O na malha de controle). Ao
 * fechar a sessão calcula TempoFinal - TempoInicial e gera o pacote para
 * POST /api/sessions/navigation ou para o arquivo de fallback.
 *
 * Lógica pura (sem Arduino/ESP): roda em teste nativo.
 * Ocupa ~12 KB: declare como variável global/estática, nunca dentro de uma task.
 * Único escritor: a task de navegação (record). A Task_Comm só lê depois de
 * state() == SessionState::Closed.
 */

#include <cstddef>
#include <cstdint>

#include "hal/ISessionStorage.h"

#ifndef MM_SESSION_CAPACITY
#define MM_SESSION_CAPACITY 1024   /* 1024 pontos * 12 bytes = 12 KB */
#endif

namespace mm {

/** Uma célula da rota. x = coluna, y = linha (convenção do PositionTracker). */
struct RoutePoint {
    uint32_t t_ms;       /* ms desde o início da sessão        */
    int16_t  x;          /* coluna (tracking: col)             */
    int16_t  y;          /* linha  (tracking: row)             */
    uint8_t  heading;    /* 0=N, 1=E, 2=S, 3=W                 */
};
static_assert(sizeof(RoutePoint) == 12, "RoutePoint deve ter 12 bytes");

enum class SessionState : uint8_t { Idle, Running, Closed };

enum class EndReason : uint8_t {
    FinishLine,   /* cruzou a linha de chegada     */
    ManualStop,   /* parada acionada pelo operador */
    Error         /* parada por falha/emergência   */
};

/** Callback de escrita em blocos (socket, arquivo, teste). Retorna 0 se ok. */
using WriteFn = int (*)(const char *chunk, size_t len, void *ctx);

class SessionBuffer {
public:
    static constexpr size_t kCapacity = MM_SESSION_CAPACITY;

    SessionBuffer() { reset(); }

    /**
     * Abre a sessão. false se já houver uma em andamento.
     * tipo_labirinto deve ser 4x4, 8x4 ou 12x4; valores desconhecidos são
     * normalizados para "desconhecido" em vez de gerar metadados inválidos.
     */
    bool start(uint32_t session_id, uint32_t now_ms,
               const char *tipo_labirinto = "desconhecido", uint16_t tentativa = 1);

    /** Define o consumo medido de bateria em mAh. Valor negativo = indisponível. */
    void setBatteryConsumedMah(float consumo_mAh);

    /**
     * Registra uma célula da rota. Não bloqueia, não aloca, não faz I/O.
     * Se o buffer encher, descarta metade dos pontos e dobra o stride: a rota
     * inteira é preservada, com menos resolução.
     */
    bool record(uint32_t now_ms, int16_t x, int16_t y, uint8_t heading);

    /** Fecha a sessão e calcula total = now_ms - start_ms. */
    bool stop(uint32_t now_ms, EndReason reason);

    SessionState state() const { return state_; }
    EndReason    reason() const { return reason_; }
    uint32_t     sessionId() const { return session_id_; }
    uint32_t     startMs() const { return start_ms_; }
    uint32_t     totalTimeMs() const { return total_time_ms_; }
    size_t       count() const { return count_; }
    uint32_t     stride() const { return stride_; }
    const RoutePoint &point(size_t i) const { return points_[i]; }

    static const char *reasonStr(EndReason r);

    /**
     * Formato do POST /api/sessions/navigation:
     * {"session_id":..,"tempo_total_ms":..,"motivo":"CHEGADA",
     *  "tipo_labirinto":"8x4","tentativa":1,"desafio_cumprido":true,
     *  "consumo_bateria_mah":null,"stride":..,"pontos":[...]}
     * Só funciona com a sessão fechada.
     */
    bool writeJson(WriteFn write, void *ctx) const;

    /** Fallback offline: begin -> writeJson -> commit (abort se algo falhar). */
    bool saveFallback(ISessionStorage &storage) const;

private:
    void reset();
    void compact();

    SessionState state_;
    EndReason    reason_;
    uint32_t session_id_;
    uint32_t start_ms_;
    uint32_t end_ms_;
    uint32_t total_time_ms_;
    uint32_t stride_;
    uint32_t seen_;        /* células recebidas antes da decimação */
    size_t   count_;
    char     tipo_labirinto_[16];
    uint16_t tentativa_;
    float    consumo_bateria_mah_;
    RoutePoint points_[kCapacity];
};

}  // namespace mm
