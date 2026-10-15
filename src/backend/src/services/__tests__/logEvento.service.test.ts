import {
  createLogEventoService,
  LogInvalidoError,
  type LogEventoDb,
  type LogEventoRow
} from "../logEvento.service";

function criarDbFake(linhas: LogEventoRow[] = []) {
  const create = jest.fn(async ({ data }) => ({ id_log: 1, ...data }) as LogEventoRow);
  const findMany = jest.fn(async () => linhas);
  const db = { logEvento: { create, findMany } } as unknown as LogEventoDb;
  return { db, create, findMany };
}

const base = { idSessao: 1, nivel: "INFO", descricao: "Início", timestampMs: 1200 };

describe("registrarLog", () => {
  it("grava o log e converte timestamp BigInt para number", async () => {
    const { db, create } = criarDbFake();
    const service = createLogEventoService(db);

    const resultado = await service.registrarLog(base);

    expect(create).toHaveBeenCalledWith({
      data: { id_sessao: 1, timestamp_ms: 1200n, tipo_evento: "INFO", descricao: "Início" }
    });
    expect(resultado).toEqual({
      id_log: 1,
      timestamp_ms: 1200,
      tipo_evento: "INFO",
      descricao: "Início"
    });
    expect(() => JSON.stringify(resultado)).not.toThrow();
  });

  it.each(["INFO", "WARN", "ERROR", "SUCCESS"])("aceita o nível %s", async (nivel) => {
    const { db } = criarDbFake();
    await expect(
      createLogEventoService(db).registrarLog({ ...base, nivel })
    ).resolves.toBeDefined();
  });

  it("rejeita WARNING (o padrão do projeto é WARN)", async () => {
    const { db, create } = criarDbFake();
    await expect(
      createLogEventoService(db).registrarLog({ ...base, nivel: "WARNING" })
    ).rejects.toBeInstanceOf(LogInvalidoError);
    expect(create).not.toHaveBeenCalled();
  });

  it.each([
    ["session_id zero", { idSessao: 0 }],
    ["session_id NaN", { idSessao: NaN }],
    ["mensagem vazia", { descricao: "   " }],
    ["t_ms negativo", { timestampMs: -1 }],
    ["t_ms NaN", { timestampMs: NaN }]
  ])("rejeita %s", async (_nome, alteracao) => {
    const { db, create } = criarDbFake();
    await expect(
      createLogEventoService(db).registrarLog({ ...base, ...alteracao })
    ).rejects.toBeInstanceOf(LogInvalidoError);
    expect(create).not.toHaveBeenCalled();
  });

  it("descarta casas decimais de t_ms", async () => {
    const { db, create } = criarDbFake();
    await createLogEventoService(db).registrarLog({ ...base, timestampMs: 1200.9 });
    expect(create.mock.calls[0][0].data.timestamp_ms).toBe(1200n);
  });
});

describe("listarLogs", () => {
  const linhas: LogEventoRow[] = [
    { id_log: 1, id_sessao: 5, timestamp_ms: 100n, tipo_evento: "INFO", descricao: "a" },
    { id_log: 2, id_sessao: 5, timestamp_ms: 250n, tipo_evento: "ERROR", descricao: "b" }
  ];

  it("lista em ordem cronológica convertendo BigInt", async () => {
    const { db, findMany } = criarDbFake(linhas);

    const resultado = await createLogEventoService(db).listarLogs(5);

    expect(findMany).toHaveBeenCalledWith({
      where: { id_sessao: 5 },
      orderBy: { timestamp_ms: "asc" }
    });
    expect(resultado.map((l) => l.timestamp_ms)).toEqual([100, 250]);
  });

  it("filtra por nível", async () => {
    const { db, findMany } = criarDbFake([linhas[1]]);

    await createLogEventoService(db).listarLogs(5, { nivel: "ERROR" });

    expect(findMany).toHaveBeenCalledWith({
      where: { id_sessao: 5, tipo_evento: "ERROR" },
      orderBy: { timestamp_ms: "asc" }
    });
  });

  it("rejeita nível inválido no filtro", async () => {
    const { db, findMany } = criarDbFake();
    await expect(
      createLogEventoService(db).listarLogs(5, { nivel: "DEBUG" })
    ).rejects.toBeInstanceOf(LogInvalidoError);
    expect(findMany).not.toHaveBeenCalled();
  });
});
