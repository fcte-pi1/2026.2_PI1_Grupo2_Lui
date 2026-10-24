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
  const calls = { create: 0, update: 0, deleteMany: 0, createMany: 0, findMany: 0 };
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
        calls.create += 1;
        const row: SessaoNavigationRow = { id_sessao: nextId++, ...data };
        sessoes.set(row.id_sessao, row);
        return row;
      },
      async update({ where, data }) {
        calls.update += 1;
        const old = sessoes.get(where.id_sessao);
        if (!old) throw new Error("sessão não encontrada");
        const row = { ...old, ...data };
        sessoes.set(row.id_sessao, row);
        return row;
      },
    },
    pontoPercurso: {
      async deleteMany({ where }) {
        calls.deleteMany += 1;
        const count = pontos.get(where.id_sessao)?.length ?? 0;
        pontos.delete(where.id_sessao);
        return { count };
      },
      async createMany({ data }) {
        calls.createMany += 1;
        if (data.length) {
          const sessionId = data[0].id_sessao;
          pontos.set(sessionId, data.map((p, id) => ({ id_ponto: BigInt(id + 1), ...p })));
        }
        return { count: data.length };
      },
      async findMany({ where }) {
        calls.findMany += 1;
        return pontos.get(where.id_sessao) ?? [];
      },
    },
  };
  const db: SessionNavigationDb = {
    async $transaction(callback) { return callback(tx); },
  };
  return { db, sessoes, pontos, calls };
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

async function rejeitaSessao(input: unknown, mensagem?: RegExp, statusCode?: number) {
  await assert.rejects(
    () => createSessionNavigationService(criarDbFake().db).iniciarSessao(input),
    (error: unknown) => error instanceof SessionNavigationError && (!mensagem || mensagem.test(error.message)) && (statusCode === undefined || error.statusCode === statusCode),
  );
}
async function rejeitaFinalizacao(input: unknown, mensagem?: RegExp) {
  await assert.rejects(
    () => createSessionNavigationService(criarDbFake().db).finalizarSessao(input),
    (error: unknown) => error instanceof SessionNavigationError && (!mensagem || mensagem.test(error.message)),
  );
}

describe("iniciarSessao — criação, validação e idempotência", () => {
  it("cria sessão com metadados e retorna a mesma sessão em POST repetido", async () => {
    const { db, sessoes, calls } = criarDbFake();
    const service = createSessionNavigationService(db);
    const a = await service.iniciarSessao({ device_session_id: 9001, tipo_labirinto: "8x4", tentativa: 2 });
    const b = await service.iniciarSessao({ device_session_id: 9001, tipo_labirinto: "8x4", tentativa: 2 });
    assert.equal(a.id_sessao, b.id_sessao);
    assert.equal(sessoes.size, 1);
    assert.equal(a.status_conclusao, "EM_ANDAMENTO");
    assert.equal(a.tipo_labirinto, "8x4");
    assert.equal(a.numero_tentativa, 2);
    assert.equal(calls.create, 1);
  });

  it("aceita tipo ausente, null, vazio ou desconhecido e tentativa ausente/null", async () => {
    const casos: unknown[] = [undefined, null, "", "desconhecido"];
    for (let i = 0; i < casos.length; i += 1) {
      const { db } = criarDbFake();
      const tipo_labirinto = casos[i];
      const tentativa = i % 2 === 0 ? undefined : null;
      const row = await createSessionNavigationService(db).iniciarSessao({
        device_session_id: i + 1,
        tipo_labirinto,
        tentativa,
      });
      assert.equal(row.tipo_labirinto, null);
      assert.equal(row.numero_tentativa, null);
    }
  });

  it("rejeita entradas não objetais e identificadores fora da faixa", async () => {
    for (const input of [null, undefined, [], "session", 10]) {
      await rejeitaSessao(input, /corpo da sessão inválido/);
    }
    for (const device_session_id of [undefined, null, "1", 1.5, 0, -1, 0x1_0000_0000]) {
      await rejeitaSessao({ device_session_id }, /inteiro positivo de 32 bits/);
    }
  });

  it("rejeita tipo de labirinto e tentativa fora dos valores permitidos", async () => {
    for (const tipo_labirinto of [5, false, "6x6"]) {
      await rejeitaSessao({ device_session_id: 1, tipo_labirinto }, /tipo_labirinto inválido/);
    }
    for (const tentativa of ["1", 1.2, 0, -1, 65536]) {
      await rejeitaSessao({ device_session_id: 1, tentativa }, /tentativa deve ser um inteiro/);
    }
    const { db } = criarDbFake();
    const min = await createSessionNavigationService(db).iniciarSessao({ device_session_id: 1, tentativa: 1, tipo_labirinto: "4x4" });
    assert.equal(min.numero_tentativa, 1);
    assert.equal(min.tipo_labirinto, "4x4");
    const { db: dbMax } = criarDbFake();
    const max = await createSessionNavigationService(dbMax).iniciarSessao({ device_session_id: 2, tentativa: 65535, tipo_labirinto: "12x4" });
    assert.equal(max.numero_tentativa, 65535);
  });

  it("retorna 409 quando o ID já pertence a uma sessão encerrada", async () => {
    const { db } = criarDbFake();
    const service = createSessionNavigationService(db);
    await service.iniciarSessao({ device_session_id: 77 });
    await service.finalizarSessao({ ...payload, session_id: 77 });
    await assert.rejects(
      () => service.iniciarSessao({ device_session_id: 77 }),
      (error: unknown) => error instanceof SessionNavigationError && error.statusCode === 409,
    );
  });
});

describe("finalizarSessao — validação e persistência", () => {
  it("atualiza uma sessão existente, persiste pontos e aceita aliases de orientação", async () => {
    const { db, sessoes, pontos, calls } = criarDbFake();
    const service = createSessionNavigationService(db);
    const inicio = await service.iniciarSessao({ device_session_id: 9001, tipo_labirinto: "8x4", tentativa: 2 });
    const entrada = {
      ...payload,
      pontos: [
        { t: 0, x: 0, y: 0, heading: "N" },
        { t: 0, x: 32767, y: 32767, heading: "NORTH" },
        { t: 2, x: 1, y: 2, heading: "e" },
        { t: 3, x: 2, y: 3, heading: "EAST" },
        { t: 4, x: 3, y: 4, heading: "s" },
        { t: 5, x: 4, y: 5, heading: "SOUTH" },
        { t: 6, x: 5, y: 6, heading: "w" },
        { t: 7, x: 6, y: 7, heading: "WEST" },
      ],
    };
    const resultado = await service.finalizarSessao(entrada);
    const saved = sessoes.get(inicio.id_sessao)!;
    assert.deepEqual(resultado, { id_sessao: inicio.id_sessao, status_conclusao: "CONCLUIDO", pontos_registrados: 8 });
    assert.equal(saved.tempo_total_segundos, 3.45);
    assert.equal(saved.desafio_cumprido, true);
    assert.equal(saved.consumo_bateria_mah, 14.25);
    assert.equal(calls.update, 1);
    assert.equal(calls.deleteMany, 1);
    assert.equal(calls.createMany, 1);
    assert.equal(pontos.get(inicio.id_sessao)?.[0].orientacao, "NORTH");
    assert.equal(pontos.get(inicio.id_sessao)?.[2].orientacao, "EAST");
    assert.equal(pontos.get(inicio.id_sessao)?.[4].orientacao, "SOUTH");
    assert.equal(pontos.get(inicio.id_sessao)?.[6].orientacao, "WEST");
  });

  it("cria sessão quando recebe finalização sem um início registrado", async () => {
    const { db, sessoes, calls } = criarDbFake();
    const result = await createSessionNavigationService(db).finalizarSessao({
      session_id: 55,
      tempo_total_ms: 2000,
      motivo: "PARADA_MANUAL",
      stride: 1,
      pontos: [{ t: 0, x: 0, y: 0, heading: "W" }],
    });
    const saved = sessoes.get(result.id_sessao)!;
    assert.equal(saved.id_dispositivo, 55n);
    assert.equal(saved.status_conclusao, "INTERROMPIDO");
    assert.equal(saved.tempo_total_segundos, 2);
    assert.equal(saved.desafio_cumprido, false);
    assert.equal(saved.tipo_labirinto, null);
    assert.equal(saved.numero_tentativa, null);
    assert.equal(saved.consumo_bateria_mah, null);
    assert.equal(calls.create, 1);
    assert.equal(calls.update, 0);
  });

  it("mapeia todos os motivos e respeita desafio_cumprido explícito", async () => {
    const casos = [
      { motivo: "CHEGADA", esperado: "CONCLUIDO", desafio: true },
      { motivo: "PARADA_MANUAL", esperado: "INTERROMPIDO", desafio: false },
      { motivo: "ERRO", esperado: "INTERROMPIDO_POR_FALHA", desafio: true },
    ] as const;
    for (const [index, caso] of casos.entries()) {
      const { db, sessoes } = criarDbFake();
      const result = await createSessionNavigationService(db).finalizarSessao({
        session_id: 100 + index,
        tempo_total_ms: 0,
        motivo: caso.motivo,
        stride: 1,
        pontos: [{ t: 0, x: 0, y: 0, heading: "NORTH" }],
        desafio_cumprido: caso.desafio,
        consumo_bateria_mah: null,
      });
      assert.equal(result.status_conclusao, caso.esperado);
      assert.equal(sessoes.get(result.id_sessao)?.desafio_cumprido, caso.desafio);
    }
    const { db } = criarDbFake();
    const fallback = await createSessionNavigationService(db).finalizarSessao({
      session_id: 200, tempo_total_ms: 0, motivo: "CHEGADA", stride: 1,
      pontos: [{ t: 0, x: 0, y: 0, heading: "N" }],
    });
    assert.equal(fallback.status_conclusao, "CONCLUIDO");
  });

  it("atualiza somente os metadados fornecidos e permite pontos com timestamp repetido", async () => {
    const { db, sessoes } = criarDbFake();
    const service = createSessionNavigationService(db);
    const start = await service.iniciarSessao({ device_session_id: 301, tipo_labirinto: "4x4", tentativa: 4 });
    const result = await service.finalizarSessao({
      session_id: 301, tempo_total_ms: 500, motivo: "ERRO", stride: 2,
      pontos: [{ t: 0, x: 0, y: 0, heading: "S" }, { t: 0, x: 0, y: 1, heading: "SOUTH" }],
      tipo_labirinto: null,
      tentativa: 10,
      desafio_cumprido: false,
      consumo_bateria_mah: 0,
    });
    const saved = sessoes.get(start.id_sessao)!;
    assert.equal(result.pontos_registrados, 2);
    assert.equal(saved.tipo_labirinto, "4x4", "null é normalizado como campo opcional ausente");
    assert.equal(saved.numero_tentativa, 10);
    assert.equal(saved.consumo_bateria_mah, 0);
    const { db: db2, sessoes: sessoes2 } = criarDbFake();
    const srv2 = createSessionNavigationService(db2);
    const start2 = await srv2.iniciarSessao({ device_session_id: 302, tipo_labirinto: "4x4", tentativa: 4 });
    await srv2.finalizarSessao({ session_id: 302, tempo_total_ms: 0, motivo: "ERRO", stride: 1, pontos: [{ t: 0, x: 0, y: 0, heading: "E" }] });
    assert.equal(sessoes2.get(start2.id_sessao)?.tipo_labirinto, "4x4");
    assert.equal(sessoes2.get(start2.id_sessao)?.numero_tentativa, 4);
  });

  it("substitui os pontos em retries em vez de duplicá-los", async () => {
    const { db, pontos } = criarDbFake();
    const service = createSessionNavigationService(db);
    await service.finalizarSessao(payload);
    await service.finalizarSessao(payload);
    assert.equal(pontos.get(1)?.length, 3);
  });

  it("rejeita corpos, IDs, tempos, motivos, stride e listas de pontos inválidos", async () => {
    await rejeitaFinalizacao(null, /corpo da sessão inválido/);
    for (const session_id of [undefined, "1", 0, -1, 1.2, 0x1_0000_0000]) {
      await rejeitaFinalizacao({ ...payload, session_id }, /inteiro positivo de 32 bits/);
    }
    for (const tempo_total_ms of [undefined, "100", -1, 0x1_0000_0000, 1.2]) {
      await rejeitaFinalizacao({ ...payload, tempo_total_ms }, /tempo_total_ms inválido/);
    }
    for (const motivo of [undefined, 5, "INVALIDO"]) {
      await rejeitaFinalizacao({ ...payload, motivo }, /motivo inválido/);
    }
    for (const stride of [undefined, 0, -1, 1.5, "1"]) {
      await rejeitaFinalizacao({ ...payload, stride }, /stride deve ser um inteiro positivo/);
    }
    for (const pontos of [undefined, null, {}, [], Array.from({ length: 100_001 }, () => ({ t: 0, x: 0, y: 0, heading: "N" }))]) {
      await rejeitaFinalizacao({ ...payload, pontos }, /pontos deve conter/);
    }
  });

  it("rejeita cada tipo de ponto inválido e timestamps decrescentes", async () => {
    const badPoints: Array<{ puntos: unknown; message: RegExp }> = [
      { puntos: [null], message: /pontos\[0\] inválido/ },
      { puntos: [], message: /pontos deve conter/ },
      { puntos: [{ t: -1, x: 0, y: 0, heading: "N" }], message: /\.t inválido/ },
      { puntos: [{ t: 1.2, x: 0, y: 0, heading: "N" }], message: /\.t inválido/ },
      { puntos: [{ t: Number.MAX_SAFE_INTEGER + 1, x: 0, y: 0, heading: "N" }], message: /\.t inválido/ },
      { puntos: [{ t: 0, x: -1, y: 0, heading: "N" }], message: /coordenadas inválidas/ },
      { puntos: [{ t: 0, x: 32768, y: 0, heading: "N" }], message: /coordenadas inválidas/ },
      { puntos: [{ t: 0, x: 0, y: -1, heading: "N" }], message: /coordenadas inválidas/ },
      { puntos: [{ t: 0, x: 0, y: 32768, heading: "N" }], message: /coordenadas inválidas/ },
      { puntos: [{ t: 0, x: 0, y: 0, heading: 4 }], message: /orientação inválida/ },
      { puntos: [{ t: 0, x: 0, y: 0, heading: "NE" }], message: /orientação inválida/ },
    ];
    for (const { puntos, message } of badPoints) {
      await rejeitaFinalizacao({ ...payload, pontos: puntos }, message);
    }
    await rejeitaFinalizacao({ ...payload, pontos: [{ t: 20, x: 0, y: 0, heading: "N" }, { t: 19, x: 1, y: 0, heading: "E" }] }, /ordem não decrescente/);
  });

  it("valida metadados opcionais da finalização", async () => {
    for (const tipo_labirinto of ["6x6", 5, false]) {
      await rejeitaFinalizacao({ ...payload, tipo_labirinto }, /tipo_labirinto inválido/);
    }
    for (const tentativa of [0, 65536, 1.1, "2"]) {
      await rejeitaFinalizacao({ ...payload, tentativa }, /tentativa deve ser um inteiro/);
    }
    for (const desafio_cumprido of ["true", 0, null]) {
      await rejeitaFinalizacao({ ...payload, desafio_cumprido }, /desafio_cumprido deve ser booleano/);
    }
    for (const consumo_bateria_mah of [-1, "5", Number.NaN, Number.POSITIVE_INFINITY]) {
      await rejeitaFinalizacao({ ...payload, consumo_bateria_mah }, /consumo_bateria_mah deve ser/);
    }
    for (const [index, consumo_bateria_mah] of [undefined, null, 0, 12.5].entries()) {
      const { db } = criarDbFake();
      const input = { ...payload, session_id: 500 + index, consumo_bateria_mah };
      const result = await createSessionNavigationService(db).finalizarSessao(input);
      assert.ok(result.id_sessao > 0);
    }
  });
});

describe("listarPontos", () => {
  it("valida ID da sessão e retorna 404 quando a sessão não existe", async () => {
    const service = createSessionNavigationService(criarDbFake().db);
    for (const id of [0, -1, 1.5, Number.NaN]) {
      await assert.rejects(() => service.listarPontos(id), SessionNavigationError);
    }
    await assert.rejects(
      () => service.listarPontos(500),
      (error: unknown) => error instanceof SessionNavigationError && error.statusCode === 404,
    );
  });

  it("retorna caminho em ordem com BigInt convertido para number e JSON seguro", async () => {
    const { db } = criarDbFake();
    const service = createSessionNavigationService(db);
    await service.finalizarSessao(payload);
    const result = await service.listarPontos(1);
    assert.deepEqual(result.map((p) => p.timestamp_ms), [0, 1200, 3000]);
    assert.doesNotThrow(() => JSON.stringify(result));
    assert.equal(result[0].orientacao, "EAST");
  });

  it("retorna lista vazia para sessão existente sem pontos", async () => {
    const { db } = criarDbFake();
    const service = createSessionNavigationService(db);
    await service.iniciarSessao({ device_session_id: 919 });
    assert.deepEqual(await service.listarPontos(1), []);
  });
});
