import { describe, it } from "node:test";
import assert from "node:assert/strict";
import {
  createLogEventoService,
  LogInvalidoError,
  type LogEventoDb,
  type LogEventoRow
} from "../logEvento.service";

function criarDbFake(linhas: LogEventoRow[] = []) {
  const chamadasCreate: Array<{
    id_sessao: number;
    timestamp_ms: bigint;
    tipo_evento: string;
    descricao: string;
  }> = [];

  const chamadasFindMany: Array<{
    where: { id_sessao: number; tipo_evento?: string };
    orderBy: { timestamp_ms: "asc" | "desc" };
    take?: number;
  }> = [];

  const create = async ({
    data
  }: {
    data: {
      id_sessao: number;
      timestamp_ms: bigint;
      tipo_evento: string;
      descricao: string;
    };
  }) => {
    chamadasCreate.push(data);
    return { id_log: 1, ...data } as LogEventoRow;
  };

  const findMany = async (args: {
    where: { id_sessao: number; tipo_evento?: string };
    orderBy: { timestamp_ms: "asc" | "desc" };
    take?: number;
  }) => {
    chamadasFindMany.push(args);
    return linhas;
  };

  const db = { logEvento: { create, findMany } } as unknown as LogEventoDb;
  return { db, chamadasCreate, chamadasFindMany };
}

const base = { idSessaoDispositivo: 1, nivel: "INFO", descricao: "Início", timestampMs: 1200 };

describe("registrarLog", () => {
  it("grava o log e converte timestamp BigInt para number", async () => {
    const { db, chamadasCreate } = criarDbFake();
    const service = createLogEventoService(db);

    const resultado = await service.registrarLog(base);

    assert.equal(chamadasCreate.length, 1);
    assert.deepEqual(chamadasCreate[0], {
      id_sessao: 1,
      timestamp_ms: 1200n,
      tipo_evento: "INFO",
      descricao: "Início"
    });
    assert.deepEqual(resultado, {
      id_log: 1,
      timestamp_ms: 1200,
      tipo_evento: "INFO",
      descricao: "Início"
    });
    assert.doesNotThrow(() => JSON.stringify(resultado));
  });

  const niveisValidos = ["INFO", "WARN", "ERROR", "SUCCESS"];
  for (const nivel of niveisValidos) {
    it(`aceita o nível ${nivel}`, async () => {
      const { db } = criarDbFake();
      const res = await createLogEventoService(db).registrarLog({ ...base, nivel });
      assert.ok(res);
    });
  }

  it("rejeita WARNING (o padrão do projeto é WARN)", async () => {
    const { db, chamadasCreate } = criarDbFake();
    await assert.rejects(
      async () => createLogEventoService(db).registrarLog({ ...base, nivel: "WARNING" }),
      LogInvalidoError
    );
    assert.equal(chamadasCreate.length, 0);
  });

  const casosRejeicao: Array<[string, Partial<typeof base>]> = [
    ["session_id zero", { idSessaoDispositivo: 0 }],
    ["session_id NaN", { idSessaoDispositivo: NaN }],
    ["mensagem vazia", { descricao: "   " }],
    ["t_ms negativo", { timestampMs: -1 }],
    ["t_ms NaN", { timestampMs: NaN }]
  ];

  for (const [nome, alteracao] of casosRejeicao) {
    it(`rejeita ${nome}`, async () => {
      const { db, chamadasCreate } = criarDbFake();
      await assert.rejects(
        async () => createLogEventoService(db).registrarLog({ ...base, ...alteracao }),
        LogInvalidoError
      );
      assert.equal(chamadasCreate.length, 0);
    });
  }

  it("descarta casas decimais de t_ms", async () => {
    const { db, chamadasCreate } = criarDbFake();
    await createLogEventoService(db).registrarLog({ ...base, timestampMs: 1200.9 });
    assert.equal(chamadasCreate[0].timestamp_ms, 1200n);
  });
});

describe("associação de logs à sessão do dispositivo", () => {
  it("resolve session_id do firmware para a chave interna do banco", async () => {
    const { db, chamadasCreate } = criarDbFake();
    (db as unknown as { sessaoNavegacao: { findUnique: (args: { where: { id_dispositivo: bigint } }) => Promise<{ id_sessao: number } | null> } }).sessaoNavegacao = {
      async findUnique({ where }) {
        if (where.id_dispositivo === 77n) return { id_sessao: 5 };
        return null;
      },
    };
    await createLogEventoService(db).registrarLog({ ...base, idSessaoDispositivo: 77 });
    assert.equal(chamadasCreate[0].id_sessao, 5);
  });
});

describe("listarLogs", () => {
  const linhas: LogEventoRow[] = [
    { id_log: 1, id_sessao: 5, timestamp_ms: 100n, tipo_evento: "INFO", descricao: "a" },
    { id_log: 2, id_sessao: 5, timestamp_ms: 250n, tipo_evento: "ERROR", descricao: "b" }
  ];

  it("lista em ordem cronológica convertendo BigInt", async () => {
    const { db, chamadasFindMany } = criarDbFake(linhas);

    const resultado = await createLogEventoService(db).listarLogs(5);

    assert.equal(chamadasFindMany.length, 1);
    assert.deepEqual(chamadasFindMany[0], {
      where: { id_sessao: 5 },
      orderBy: { timestamp_ms: "asc" }
    });
    assert.deepEqual(
      resultado.map((l) => l.timestamp_ms),
      [100, 250]
    );
  });

  it("consulta os eventos recentes sem alterar a ordem cronológica da resposta", async () => {
    const recentesPrimeiro: LogEventoRow[] = [
      { id_log: 3, id_sessao: 5, timestamp_ms: 500n, tipo_evento: "ERROR", descricao: "c" },
      { id_log: 2, id_sessao: 5, timestamp_ms: 250n, tipo_evento: "WARN", descricao: "b" }
    ];
    const { db, chamadasFindMany } = criarDbFake(recentesPrimeiro);
    const resultado = await createLogEventoService(db).listarLogs(5, { limite: 2, recentes: true });

    assert.deepEqual(chamadasFindMany[0], {
      where: { id_sessao: 5 },
      orderBy: { timestamp_ms: "desc" },
      take: 2
    });
    assert.deepEqual(resultado.map((l) => l.timestamp_ms), [250, 500]);
  });

  it("rejeita limite inválido", async () => {
    const { db, chamadasFindMany } = criarDbFake();
    await assert.rejects(
      async () => createLogEventoService(db).listarLogs(5, { limite: 0 }),
      LogInvalidoError
    );
    assert.equal(chamadasFindMany.length, 0);
  });

  it("filtra por nível", async () => {
    const { db, chamadasFindMany } = criarDbFake([linhas[1]]);

    await createLogEventoService(db).listarLogs(5, { nivel: "ERROR" });

    assert.equal(chamadasFindMany.length, 1);
    assert.deepEqual(chamadasFindMany[0], {
      where: { id_sessao: 5, tipo_evento: "ERROR" },
      orderBy: { timestamp_ms: "asc" }
    });
  });

  it("rejeita nível inválido no filtro", async () => {
    const { db, chamadasFindMany } = criarDbFake();
    await assert.rejects(
      async () => createLogEventoService(db).listarLogs(5, { nivel: "DEBUG" }),
      LogInvalidoError
    );
    assert.equal(chamadasFindMany.length, 0);
  });
});
