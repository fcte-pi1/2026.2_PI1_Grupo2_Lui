import { describe, it } from "node:test";
import assert from "node:assert/strict";
import {
  createLogEventoService,
  isNivelLog,
  LogInvalidoError,
  NIVEIS_LOG,
  type LogEventoDb,
  type LogEventoRow,
} from "../logEvento.service";

function criarDbFake(linhas: LogEventoRow[] = [], comSessao?: { id_sessao: number } | null) {
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
  const create = async ({ data }: { data: {
    id_sessao: number;
    timestamp_ms: bigint;
    tipo_evento: string;
    descricao: string;
  } }) => {
    chamadasCreate.push(data);
    return { id_log: chamadasCreate.length, ...data } as LogEventoRow;
  };
  const findMany = async (args: {
    where: { id_sessao: number; tipo_evento?: string };
    orderBy: { timestamp_ms: "asc" | "desc" };
    take?: number;
  }) => {
    chamadasFindMany.push(args);
    return linhas;
  };
  const rawDb: Record<string, unknown> = { logEvento: { create, findMany } };
  if (arguments.length > 1) {
    rawDb.sessaoNavegacao = {
      async findUnique({ where }: { where: { id_dispositivo: bigint } }) {
        return where.id_dispositivo === 77n ? comSessao ?? null : null;
      },
    };
  }
  return { db: rawDb as unknown as LogEventoDb, chamadasCreate, chamadasFindMany };
}

const base = { idSessaoDispositivo: 1, nivel: "INFO", descricao: "Início", timestampMs: 1200 };
const linhas: LogEventoRow[] = [
  { id_log: 1, id_sessao: 5, timestamp_ms: 100n, tipo_evento: "INFO", descricao: "a" },
  { id_log: 2, id_sessao: 5, timestamp_ms: 250n, tipo_evento: "ERROR", descricao: "b" },
];

function criaLinha(id: number, timestamp: bigint, tipo_evento = "INFO"): LogEventoRow {
  return { id_log: id, id_sessao: 5, timestamp_ms: timestamp, tipo_evento, descricao: `evento ${id}` };
}

describe("isNivelLog", () => {
  it("reconhece todos os níveis suportados e rejeita outros tipos/valores", () => {
    for (const nivel of NIVEIS_LOG) assert.equal(isNivelLog(nivel), true);
    for (const nivel of [undefined, null, 2, {}, "DEBUG", "WARNING", ""]) assert.equal(isNivelLog(nivel), false);
  });
});

describe("registrarLog", () => {
  it("persiste, normaliza descrição e converte BigInt para number", async () => {
    const { db, chamadasCreate } = criarDbFake();
    const resultado = await createLogEventoService(db).registrarLog({ ...base, descricao: "  Início  " });
    assert.deepEqual(chamadasCreate[0], {
      id_sessao: 1,
      timestamp_ms: 1200n,
      tipo_evento: "INFO",
      descricao: "Início",
    });
    assert.deepEqual(resultado, { id_log: 1, timestamp_ms: 1200, tipo_evento: "INFO", descricao: "Início" });
    assert.doesNotThrow(() => JSON.stringify(resultado));
  });

  it("aceita todos os níveis e timestamp fracionário não negativo", async () => {
    for (const nivel of NIVEIS_LOG) {
      const { db, chamadasCreate } = criarDbFake();
      const result = await createLogEventoService(db).registrarLog({ ...base, nivel, timestampMs: 0.9 });
      assert.equal(result.tipo_evento, nivel);
      assert.equal(chamadasCreate[0].timestamp_ms, 0n);
    }
  });

  it("rejeita ID inválido, nível inválido e descrição vazia ou não textual", async () => {
    const casos: Array<[string, unknown, RegExp]> = [
      ["ID não inteiro", { ...base, idSessaoDispositivo: 1.5 }, /session_id inválido/],
      ["ID zero", { ...base, idSessaoDispositivo: 0 }, /session_id inválido/],
      ["ID negativo", { ...base, idSessaoDispositivo: -1 }, /session_id inválido/],
      ["nível numérico", { ...base, nivel: 2 }, /nível inválido/],
      ["nível desconhecido", { ...base, nivel: "WARNING" }, /nível inválido/],
      ["texto vazio", { ...base, descricao: "   " }, /mensagem vazia/],
      ["texto não string", { ...base, descricao: 22 }, /mensagem vazia/],
      ["timestamp negativo", { ...base, timestampMs: -1 }, /t_ms inválido/],
      ["timestamp NaN", { ...base, timestampMs: Number.NaN }, /t_ms inválido/],
      ["timestamp infinito", { ...base, timestampMs: Number.POSITIVE_INFINITY }, /t_ms inválido/],
    ];
    for (const [nome, input, mensagem] of casos) {
      const { db, chamadasCreate } = criarDbFake();
      await assert.rejects(() => createLogEventoService(db).registrarLog(input as typeof base), (error: unknown) =>
        error instanceof LogInvalidoError && mensagem.test(error.message), nome);
      assert.equal(chamadasCreate.length, 0);
    }
  });

  it("associa o ID do dispositivo à chave interna da sessão", async () => {
    const { db, chamadasCreate } = criarDbFake([], { id_sessao: 5 });
    await createLogEventoService(db).registrarLog({ ...base, idSessaoDispositivo: 77 });
    assert.equal(chamadasCreate[0].id_sessao, 5);
  });

  it("rejeita a mensagem se o dispositivo não tem sessão registrada", async () => {
    const { db, chamadasCreate } = criarDbFake([], null);
    await assert.rejects(
      () => createLogEventoService(db).registrarLog({ ...base, idSessaoDispositivo: 77 }),
      (error: unknown) => error instanceof LogInvalidoError && /registre a sessão antes/.test(error.message),
    );
    assert.equal(chamadasCreate.length, 0);
  });

  it("propaga falhas do banco durante a criação", async () => {
    const { db } = criarDbFake();
    db.logEvento.create = async () => { throw new Error("banco indisponível"); };
    await assert.rejects(() => createLogEventoService(db).registrarLog(base), /banco indisponível/);
  });
});

describe("listarLogs", () => {
  it("lista cronologicamente sem filtro nem limite", async () => {
    const { db, chamadasFindMany } = criarDbFake(linhas);
    const resultado = await createLogEventoService(db).listarLogs(5);
    assert.deepEqual(chamadasFindMany[0], { where: { id_sessao: 5 }, orderBy: { timestamp_ms: "asc" } });
    assert.deepEqual(resultado.map((l) => l.timestamp_ms), [100, 250]);
  });

  it("filtra por nível, aplica limite e consulta os mais recentes preservando ordem cronológica na saída", async () => {
    const recentesPrimeiro = [criaLinha(3, 500n, "ERROR"), criaLinha(2, 250n, "WARN")];
    const { db, chamadasFindMany } = criarDbFake(recentesPrimeiro);
    const resultado = await createLogEventoService(db).listarLogs(5, { nivel: "WARN", limite: 2, recentes: true });
    assert.deepEqual(chamadasFindMany[0], {
      where: { id_sessao: 5, tipo_evento: "WARN" },
      orderBy: { timestamp_ms: "desc" },
      take: 2,
    });
    assert.deepEqual(resultado.map((l) => l.timestamp_ms), [250, 500]);
    assert.deepEqual(recentesPrimeiro.map((l) => l.timestamp_ms), [500n, 250n], "o array original não é mutado");
  });

  it("valida nível e limites mínimo/máximo", async () => {
    const limitesInvalidos = [0, -1, 1.5, 1001, 10000, Number.NaN];
    for (const limite of limitesInvalidos) {
      const { db, chamadasFindMany } = criarDbFake();
      await assert.rejects(() => createLogEventoService(db).listarLogs(5, { limite }), LogInvalidoError);
      assert.equal(chamadasFindMany.length, 0);
    }
    const { db, chamadasFindMany } = criarDbFake();
    await createLogEventoService(db).listarLogs(5, { limite: 1, nivel: "INFO" });
    assert.deepEqual(chamadasFindMany[0], {
      where: { id_sessao: 5, tipo_evento: "INFO" }, orderBy: { timestamp_ms: "asc" }, take: 1,
    });
    const { db: maxDb, chamadasFindMany: maxCalls } = criarDbFake();
    await createLogEventoService(maxDb).listarLogs(5, { limite: 1000 });
    assert.equal(maxCalls[0].take, 1000);
    const { db: invalidDb, chamadasFindMany: invalidCalls } = criarDbFake();
    await assert.rejects(() => createLogEventoService(invalidDb).listarLogs(5, { nivel: "TRACE" }));
    assert.equal(invalidCalls.length, 0);
  });

  it("devolve lista vazia e aceita recentes=true sem alterar array original", async () => {
    const { db, chamadasFindMany } = criarDbFake([]);
    const result = await createLogEventoService(db).listarLogs(999, { recentes: true });
    assert.deepEqual(result, []);
    assert.deepEqual(chamadasFindMany[0], { where: { id_sessao: 999 }, orderBy: { timestamp_ms: "desc" } });
  });

  it("propaga falhas do banco na consulta", async () => {
    const { db } = criarDbFake();
    db.logEvento.findMany = async () => { throw new Error("consulta falhou"); };
    await assert.rejects(() => createLogEventoService(db).listarLogs(5), /consulta falhou/);
  });
});
