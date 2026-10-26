import { afterEach, describe, it } from "node:test";
import assert from "node:assert/strict";
import type { Server } from "node:http";
import { createApp } from "../../app";
import { createLogsRouter } from "../logs.routes";
import type { LogsRouterServices } from "../logs.routes";

const activeServers: Server[] = [];
afterEach(async () => {
  await Promise.all(activeServers.splice(0).map(async (server) => {
    if (server.listening) await new Promise<void>((resolve, reject) => server.close((error) => error ? reject(error) : resolve()));
  }));
});

async function start(app: ReturnType<typeof createApp>) {
  const server = app.listen(0, "127.0.0.1");
  activeServers.push(server);
  await new Promise<void>((resolve) => server.once("listening", resolve));
  const address = server.address();
  if (!address || typeof address === "string") throw new Error("endereço de teste indisponível");
  return `http://127.0.0.1:${address.port}`;
}

function makeRouter(calls: Array<{ method: string; args: unknown[] }>) {
  const services = {
    logs: {
      async listarLogs(id: number, filters: unknown) { calls.push({ method: "listarLogs", args: [id, filters] }); return [{ id_log: 8, timestamp_ms: 20 }]; },
    },
    sessoes: {
      async listarSessoes(filters: unknown) { calls.push({ method: "listarSessoes", args: [filters] }); return [{ id_sessao: 3, status_conclusao: "CONCLUIDO" }]; },
    },
    navegacao: {
      async iniciarSessao(body: unknown) {
        calls.push({ method: "iniciarSessao", args: [body] });
        return { id_sessao: 3, id_dispositivo: 42n, data_inicio: new Date("2026-01-01T00:00:00.000Z"), status_conclusao: "EM_ANDAMENTO" };
      },
      async finalizarSessao(body: unknown) { calls.push({ method: "finalizarSessao", args: [body] }); return { id_sessao: 3, status_conclusao: "CONCLUIDO", pontos_registrados: 2 }; },
      async listarPontos(id: number) { calls.push({ method: "listarPontos", args: [id] }); return [{ ordem: 0, timestamp_ms: 0, pos_x: 1, pos_y: 2, orientacao: "NORTH" }]; },
    },
  };
  return createLogsRouter(services as unknown as LogsRouterServices);
}

describe("logs.routes", () => {
  it("encaminha os cinco endpoints para os serviços corretos", async () => {
    const calls: Array<{ method: string; args: unknown[] }> = [];
    const base = await start(createApp(makeRouter(calls)));

    const sessionsResponse = await fetch(`${base}/api/sessions?status=CONCLUIDO`);
    assert.equal(sessionsResponse.status, 200);
    assert.deepEqual(await sessionsResponse.json(), [{ id_sessao: 3, status_conclusao: "CONCLUIDO" }]);

    const startResponse = await fetch(`${base}/api/sessions`, { method: "POST", headers: { "content-type": "application/json" }, body: JSON.stringify({ device_session_id: 42 }) });
    assert.equal(startResponse.status, 201);
    assert.equal((await startResponse.json() as any).device_session_id, 42);

    const finishResponse = await fetch(`${base}/api/sessions/navigation`, { method: "POST", headers: { "content-type": "application/json" }, body: JSON.stringify({ session_id: 42, motivo: "CHEGADA" }) });
    assert.equal(finishResponse.status, 200);
    assert.equal((await finishResponse.json() as any).pontos_registrados, 2);

    const pathResponse = await fetch(`${base}/api/sessions/3/navigation`);
    assert.equal(pathResponse.status, 200);
    assert.equal((await pathResponse.json() as any[])[0].orientacao, "NORTH");

    const logsResponse = await fetch(`${base}/api/sessions/3/logs?nivel=WARN&limite=10&recentes=true`);
    assert.equal(logsResponse.status, 200);
    assert.equal((await logsResponse.json() as any[])[0].id_log, 8);

    assert.deepEqual(calls.map((call) => call.method), ["listarSessoes", "iniciarSessao", "finalizarSessao", "listarPontos", "listarLogs"]);
    assert.deepEqual(calls[0].args, [{ status: "CONCLUIDO" }]);
    assert.deepEqual(calls[3].args, [3]);
    assert.deepEqual(calls[4].args, [3, { nivel: "WARN", limite: 10, recentes: true }]);
  });

  it("roteia ids inválidos para a validação HTTP do controlador", async () => {
    const base = await start(createApp(makeRouter([])));
    const path = await fetch(`${base}/api/sessions/abc/navigation`);
    const logs = await fetch(`${base}/api/sessions/0/logs`);
    assert.equal(path.status, 400); assert.deepEqual(await path.json(), { erro: "id da sessão inválido" });
    assert.equal(logs.status, 400); assert.deepEqual(await logs.json(), { erro: "id da sessão inválido" });
  });
});
