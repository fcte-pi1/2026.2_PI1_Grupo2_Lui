import { describe, it } from "node:test";
import assert from "node:assert/strict";
import type { Request, Response } from "express";
import { createLogsController } from "../logs.controller";
import { LogInvalidoError, type LogEventoService } from "../../services/logEvento.service";
import { SessionNavigationError, type SessionNavigationService } from "../../services/sessionNavigation.service";
import type { SessionListService } from "../../services/sessionList.service";

type Endpoint = "listarSessoes" | "iniciarSessao" | "finalizarSessao" | "listarPontos" | "listarLogs";
type Override = Partial<Record<Endpoint, (...args: any[]) => any>>;
function setup(overrides: Override = {}) {
  const calls: Array<{ method: string; args: unknown[] }> = [];
  const invoke = (method: Endpoint, fallback: (...args: any[]) => any) => (...args: any[]) => {
    calls.push({ method, args });
    return overrides[method] ? overrides[method]!(...args) : fallback(...args);
  };
  const services = {
    logs: { listarLogs: invoke("listarLogs", async () => [{ id_log: 3 }]) },
    sessoes: { listarSessoes: invoke("listarSessoes", async () => [{ id_sessao: 1 }]) },
    navegacao: {
      iniciarSessao: invoke("iniciarSessao", async () => ({
        id_sessao: 5, id_dispositivo: 77n, data_inicio: new Date("2026-01-01T00:00:00Z"), status_conclusao: "EM_ANDAMENTO",
      })),
      finalizarSessao: invoke("finalizarSessao", async () => ({ id_sessao: 5, status_conclusao: "CONCLUIDO", pontos_registrados: 1 })),
      listarPontos: invoke("listarPontos", async () => [{ ordem: 0, timestamp_ms: 0, pos_x: 1, pos_y: 2, orientacao: "NORTH" }]),
    },
  };
  const controller = createLogsController(
    services.logs as unknown as LogEventoService,
    services.sessoes as unknown as SessionListService,
    services.navegacao as unknown as SessionNavigationService,
  );
  return { controller, calls };
}
function req(values: { query?: Record<string, unknown>; params?: Record<string, string>; body?: unknown } = {}) {
  return { query: values.query ?? {}, params: values.params ?? {}, body: values.body ?? {} } as unknown as Request;
}
function res() {
  const response: { statusCode: number; body: unknown; status: (code: number) => Response; json: (body: unknown) => Response } = {
    statusCode: 200,
    body: undefined,
    status(code) { this.statusCode = code; return this as unknown as Response; },
    json(body) { this.body = body; return this as unknown as Response; },
  };
  return response;
}
async function quiet<T>(action: () => Promise<T>): Promise<T> {
  const original = console.error;
  console.error = () => {};
  try { return await action(); } finally { console.error = original; }
}

describe("logs.controller — listarSessoes", () => {
  it("encaminha status string e retorna as sessões", async () => {
    const { controller, calls } = setup(); const response = res();
    await controller.listarSessoes(req({ query: { status: "CONCLUIDO" } }), response as unknown as Response);
    assert.deepEqual(calls[0].args, [{ status: "CONCLUIDO" }]);
    assert.deepEqual(response.body, [{ id_sessao: 1 }]);
  });
  it("ignora status que não seja string", async () => {
    const { controller, calls } = setup(); const response = res();
    await controller.listarSessoes(req({ query: { status: ["CONCLUIDO"] } }), response as unknown as Response);
    assert.deepEqual(calls[0].args, [{ status: undefined }]);
  });
  it("converte falha do serviço em HTTP 500", async () => {
    const { controller } = setup({ listarSessoes: async () => { throw new Error("falha"); } }); const response = res();
    await quiet(() => controller.listarSessoes(req(), response as unknown as Response));
    assert.equal(response.statusCode, 500);
  });
});

describe("logs.controller — iniciarSessao", () => {
  it("retorna HTTP 201 e converte o id BigInt para number", async () => {
    const { controller } = setup(); const response = res();
    await controller.iniciarSessao(req({ body: { device_session_id: 77 } }), response as unknown as Response);
    assert.equal(response.statusCode, 201);
    assert.equal((response.body as any).device_session_id, 77);
  });
  it("preserva device_session_id nulo", async () => {
    const { controller } = setup({ iniciarSessao: async () => ({ id_sessao: 6, id_dispositivo: null, data_inicio: new Date(0), status_conclusao: "EM_ANDAMENTO" }) }); const response = res();
    await controller.iniciarSessao(req(), response as unknown as Response);
    assert.equal((response.body as any).device_session_id, null);
  });
  it("retorna status específico para SessionNavigationError", async () => {
    const { controller } = setup({ iniciarSessao: async () => { throw new SessionNavigationError("conflito", 409); } }); const response = res();
    await controller.iniciarSessao(req(), response as unknown as Response);
    assert.equal(response.statusCode, 409); assert.deepEqual(response.body, { erro: "conflito" });
  });
  it("converte erros inesperados em HTTP 500", async () => {
    const { controller } = setup({ iniciarSessao: async () => { throw new Error("falha"); } }); const response = res();
    await quiet(() => controller.iniciarSessao(req(), response as unknown as Response));
    assert.equal(response.statusCode, 500);
  });
});

describe("logs.controller — finalizarSessao", () => {
  it("retorna a resposta do serviço com HTTP 200", async () => {
    const { controller } = setup(); const response = res();
    await controller.finalizarSessao(req({ body: { session_id: 77 } }), response as unknown as Response);
    assert.equal(response.statusCode, 200); assert.equal((response.body as any).status_conclusao, "CONCLUIDO");
  });
  it("trata SessionNavigationError", async () => {
    const { controller } = setup({ finalizarSessao: async () => { throw new SessionNavigationError("inválida", 400); } }); const response = res();
    await controller.finalizarSessao(req(), response as unknown as Response);
    assert.equal(response.statusCode, 400);
  });
  it("trata erros inesperados", async () => {
    const { controller } = setup({ finalizarSessao: async () => { throw new Error("falha"); } }); const response = res();
    await quiet(() => controller.finalizarSessao(req(), response as unknown as Response));
    assert.equal(response.statusCode, 500);
  });
});

describe("logs.controller — listarPontos", () => {
  it("rejeita id que não é inteiro", async () => {
    const { controller, calls } = setup(); const response = res();
    await controller.listarPontos(req({ params: { id: "abc" } }), response as unknown as Response);
    assert.equal(response.statusCode, 400); assert.equal(calls.length, 0);
  });
  it("rejeita id inteiro menor ou igual a zero", async () => {
    const { controller, calls } = setup(); const response = res();
    await controller.listarPontos(req({ params: { id: "0" } }), response as unknown as Response);
    assert.equal(response.statusCode, 400); assert.equal(calls.length, 0);
  });
  it("lista os pontos para id válido", async () => {
    const { controller, calls } = setup(); const response = res();
    await controller.listarPontos(req({ params: { id: "5" } }), response as unknown as Response);
    assert.equal((calls[0].args[0]), 5); assert.equal(response.statusCode, 200);
  });
  it("trata sessão inexistente", async () => {
    const { controller } = setup({ listarPontos: async () => { throw new SessionNavigationError("não encontrada", 404); } }); const response = res();
    await controller.listarPontos(req({ params: { id: "5" } }), response as unknown as Response);
    assert.equal(response.statusCode, 404);
  });
  it("trata falha inesperada", async () => {
    const { controller } = setup({ listarPontos: async () => { throw new Error("falha"); } }); const response = res();
    await quiet(() => controller.listarPontos(req({ params: { id: "5" } }), response as unknown as Response));
    assert.equal(response.statusCode, 500);
  });
});

describe("logs.controller — listarLogs", () => {
  it("encaminha filtros de query válidos", async () => {
    const { controller, calls } = setup(); const response = res();
    await controller.listarLogs(req({ params: { id: "2" }, query: { nivel: "WARN", limite: "10", recentes: "true" } }), response as unknown as Response);
    assert.deepEqual(calls[0].args, [2, { nivel: "WARN", limite: 10, recentes: true }]);
  });
  it("ignora filtros que não sejam strings e recentes diferentes de true", async () => {
    const { controller, calls } = setup(); const response = res();
    await controller.listarLogs(req({ params: { id: "2" }, query: { nivel: ["INFO"], limite: ["5"], recentes: "false" } }), response as unknown as Response);
    assert.deepEqual(calls[0].args, [2, { nivel: undefined, limite: undefined, recentes: false }]);
  });
  it("rejeita id não inteiro e id menor ou igual a zero", async () => {
    const { controller, calls } = setup(); const invalid = res();
    await controller.listarLogs(req({ params: { id: "NaN" } }), invalid as unknown as Response);
    assert.equal(invalid.statusCode, 400);
    const zero = res(); await controller.listarLogs(req({ params: { id: "-1" } }), zero as unknown as Response);
    assert.equal(zero.statusCode, 400); assert.equal(calls.length, 0);
  });
  it("converte LogInvalidoError em HTTP 400", async () => {
    const { controller } = setup({ listarLogs: async () => { throw new LogInvalidoError("filtro inválido"); } }); const response = res();
    await controller.listarLogs(req({ params: { id: "2" } }), response as unknown as Response);
    assert.equal(response.statusCode, 400);
  });
  it("converte erro inesperado em HTTP 500", async () => {
    const { controller } = setup({ listarLogs: async () => { throw new Error("falha"); } }); const response = res();
    await quiet(() => controller.listarLogs(req({ params: { id: "2" } }), response as unknown as Response));
    assert.equal(response.statusCode, 500);
  });
});
