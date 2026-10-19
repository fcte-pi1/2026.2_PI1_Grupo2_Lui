import { describe, it } from "node:test";
import assert from "node:assert/strict";
import {
  createSessionNavigationService,
  SessionNavigationError,
  type SessionNavigationDb,
  type SessionNavigationTx,
  type SessaoNavigationRow,
  type PontoPercursoRow,
} from "../sessionNavigation.service";

function criarDbFake() {
  const sessoes = new Map<number, SessaoNavigationRow>();
  const pontos = new Map<number, PontoPercursoRow[]>();
  let nextId = 1;
  const tx: SessionNavigationTx = {
    sessaoNavegacao: {
      async findUnique({ where }) {
        if (where.id_dispositivo !== undefined) {
          return [...sessoes.values()].find((s) => s.id_dispositivo === where.id_dispositivo) ?? null;
        }
        if (where.id_sessao !== undefined) return sessoes.get(where.id_sessao) ?? null;
        return null;
      },
      async create({ data }) {
        const row: SessaoNavigationRow = { id_sessao: nextId++, ...data };
        sessoes.set(row.id_sessao, row);
        return row;
      },
      async update({ where, data }) {
        const old = sessoes.get(where.id_sessao);
        if (!old) throw new Error("sessão não encontrada");
        const row = { ...old, ...data };
        sessoes.set(row.id_sessao, row);
        return row;
      },
    },
    pontoPercurso: {
      async deleteMany({ where }) { pontos.delete(where.id_sessao); return { count: 1 }; },
      async createMany({ data }) {
        if (data.length) {
          const sessionId = data[0].id_sessao;
          pontos.set(sessionId, data.map((p, id) => ({ id_ponto: BigInt(id + 1), ...p })));
        }
        return { count: data.length };
      },
      async findMany({ where }) { return pontos.get(where.id_sessao) ?? []; },
    },
  };
  const db: SessionNavigationDb = {
    async $transaction(callback) { return callback(tx); },
  };
  return { db, sessoes, pontos };
}

const payload = {
  session_id: 9001,
  tempo_total_ms: 3450,
  motivo: "CHEGADA",
  tipo_labirinto: "8x4",
  tentativa: 2,
  desafio_cumprido: true,
  consumo_bateria_mah: 14.25,
  stride: 1,
  pontos: [
    { t: 0, x: 0, y: 0, heading: "EAST" },
    { t: 1200, x: 1, y: 0, heading: "E" },
    { t: 3000, x: 1, y: 0, heading: "SOUTH" },
  ],
};

describe("iniciarSessao", () => {
  it("cria sessão em andamento com metadados e torna POST repetido idempotente", async () => {
    const { db, sessoes } = criarDbFake();
    const service = createSessionNavigationService(db);
    const a = await service.iniciarSessao({ device_session_id: 9001, tipo_labirinto: "8x4", tentativa: 2 });
    const b = await service.iniciarSessao({ device_session_id: 9001, tipo_labirinto: "8x4", tentativa: 2 });
    assert.equal(a.id_sessao, b.id_sessao);
    assert.equal(sessoes.size, 1);
    assert.equal(a.status_conclusao, "EM_ANDAMENTO");
    assert.equal(a.tipo_labirinto, "8x4");
    assert.equal(a.numero_tentativa, 2);
  });

  it("rejeita tipo de labirinto desconhecido", async () => {
    const { db } = criarDbFake();
    await assert.rejects(
      () => createSessionNavigationService(db).iniciarSessao({ device_session_id: 2, tipo_labirinto: "6x6" }),
      SessionNavigationError,
    );
  });
});

describe("finalizarSessao", () => {
  it("salva metadados e sequência ordenada, sem confundir revisita com célula do mapa", async () => {
    const { db, sessoes, pontos } = criarDbFake();
    const service = createSessionNavigationService(db);
    const inicio = await service.iniciarSessao({ device_session_id: 9001, tipo_labirinto: "8x4", tentativa: 2 });
    const resultado = await service.finalizarSessao(payload);
    const saved = sessoes.get(inicio.id_sessao)!;
    assert.deepEqual(resultado, { id_sessao: inicio.id_sessao, status_conclusao: "CONCLUIDO", pontos_registrados: 3 });
    assert.equal(saved.tempo_total_segundos, 3.45);
    assert.equal(saved.desafio_cumprido, true);
    assert.equal(saved.consumo_bateria_mah, 14.25);
    assert.equal(pontos.get(inicio.id_sessao)?.length, 3);
    assert.equal(pontos.get(inicio.id_sessao)?.[1].orientacao, "EAST");
    assert.equal(pontos.get(inicio.id_sessao)?.[2].pos_x, 1); // revisita permanece na sequência
  });

  it("recupera pontos com BigInt convertido para JSON seguro", async () => {
    const { db } = criarDbFake();
    const service = createSessionNavigationService(db);
    await service.finalizarSessao(payload);
    const result = await service.listarPontos(1);
    assert.deepEqual(result.map((p) => p.timestamp_ms), [0, 1200, 3000]);
    assert.doesNotThrow(() => JSON.stringify(result));
  });

  it("retry de finalização substitui pontos em vez de duplicá-los", async () => {
    const { db, pontos } = criarDbFake();
    const service = createSessionNavigationService(db);
    await service.finalizarSessao(payload);
    await service.finalizarSessao(payload);
    assert.equal(pontos.get(1)?.length, 3);
  });

  it("rejeita pontos fora de ordem e sessão inexistente ao ler caminho", async () => {
    const { db } = criarDbFake();
    const service = createSessionNavigationService(db);
    await assert.rejects(
      () => service.finalizarSessao({ ...payload, pontos: [{ t: 200, x: 0, y: 0, heading: "N" }, { t: 100, x: 1, y: 0, heading: "E" }] }),
      SessionNavigationError,
    );
    await assert.rejects(() => service.listarPontos(500), (error: unknown) =>
      error instanceof SessionNavigationError && error.statusCode === 404,
    );
  });
});
