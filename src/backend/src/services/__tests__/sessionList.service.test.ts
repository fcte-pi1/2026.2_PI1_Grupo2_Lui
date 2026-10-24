import { describe, it } from "node:test";
import assert from "node:assert/strict";
import { createSessionListService, type SessionListDb, type SessaoRow } from "../sessionList.service";

function makeDb(rows: SessaoRow[] = []) {
  const calls: unknown[] = [];
  const db = {
    sessaoNavegacao: {
      async findMany(args: unknown) {
        calls.push(args);
        return rows;
      },
    },
  } as unknown as SessionListDb;
  return { db, calls };
}

const row: SessaoRow = {
  id_sessao: 4,
  data_inicio: new Date("2026-01-01T00:00:00Z"),
  status_conclusao: "CONCLUIDO",
  tempo_total_segundos: 12,
  tipo_labirinto: "4x4",
  numero_tentativa: 1,
  desafio_cumprido: true,
  consumo_bateria_mah: null,
};

describe("createSessionListService", () => {
  it("lista sem filtro e seleciona apenas os campos públicos", async () => {
    const { db, calls } = makeDb([row]);
    assert.deepEqual(await createSessionListService(db).listarSessoes(), [row]);
    assert.deepEqual(calls[0], {
      orderBy: { data_inicio: "desc" },
      select: {
        id_sessao: true,
        data_inicio: true,
        status_conclusao: true,
        tempo_total_segundos: true,
        tipo_labirinto: true,
        numero_tentativa: true,
        desafio_cumprido: true,
        consumo_bateria_mah: true,
      },
    });
    assert.equal(JSON.stringify(await createSessionListService(db).listarSessoes()).includes("id_dispositivo"), false);
  });

  it("aplica o filtro de status quando ele é truthy", async () => {
    const { db, calls } = makeDb([]);
    await createSessionListService(db).listarSessoes({ status: "EM_ANDAMENTO" });
    assert.deepEqual((calls[0] as { where?: unknown }).where, { status_conclusao: "EM_ANDAMENTO" });
  });

  it("omite o filtro quando o status é uma string vazia", async () => {
    const { db, calls } = makeDb([]);
    await createSessionListService(db).listarSessoes({ status: "" });
    assert.equal(Object.hasOwn(calls[0] as object, "where"), false);
  });

  it("propaga falhas de consulta do banco", async () => {
    const error = new Error("db indisponível");
    const db = { sessaoNavegacao: { findMany: async () => { throw error; } } } as unknown as SessionListDb;
    await assert.rejects(() => createSessionListService(db).listarSessoes(), error);
  });
});
