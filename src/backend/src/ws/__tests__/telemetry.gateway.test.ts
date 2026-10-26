import { afterEach, describe, it } from "node:test";
import assert from "node:assert/strict";
import http, { type IncomingMessage } from "node:http";
import WebSocket, { WebSocketServer } from "ws";
import { connectionRole, createTelemetryGateway, sendIfOpen } from "../telemetry.gateway";
import { LogInvalidoError } from "../../services/logEvento.service";
import { SessionNavigationError } from "../../services/sessionNavigation.service";
import { TelemetryInvalidaError } from "../../services/telemetry.service";
import type { LogEventoService } from "../../services/logEvento.service";
import type { SessionNavigationService } from "../../services/sessionNavigation.service";
import type { TelemetryService } from "../../services/telemetry.service";

type GatewayServices = { logs: LogEventoService; sessions: SessionNavigationService; telemetry: TelemetryService };
type Packet = Record<string, unknown>;
type Client = { socket: WebSocket; receive: (timeoutMs?: number) => Promise<Packet> };
type ServiceOverrides = {
  startError?: Error;
  logError?: Error;
  endError?: Error;
  telemetryError?: Error;
};

function makeServices(overrides: ServiceOverrides = {}) {
  const calls: Array<{ method: string; args: unknown }> = [];
  const services = {
    logs: {
      async registrarLog(input: unknown) {
        calls.push({ method: "log", args: input });
        if (overrides.logError) throw overrides.logError;
        return { id_log: 10, timestamp_ms: 55, tipo_evento: "INFO", descricao: "ok" };
      },
    },
    sessions: {
      async iniciarSessao(input: unknown) {
        calls.push({ method: "start", args: input });
        if (overrides.startError) throw overrides.startError;
        return { id_sessao: 31 };
      },
      async finalizarSessao(input: unknown) {
        calls.push({ method: "end", args: input });
        if (overrides.endError) throw overrides.endError;
        return { id_sessao: 31, status_conclusao: "CONCLUIDO", pontos_registrados: 2 };
      },
    },
    telemetry: {
      async registrarTelemetria(input: unknown) {
        calls.push({ method: "telemetry", args: input });
        if (overrides.telemetryError) throw overrides.telemetryError;
        return {
          session_id: 77,
          timestamp_ms: 250,
          battery_voltage: 7.4,
          battery_current_a: 0.2,
          pid_error: -0.1,
          wheels_speed: { left: 0.2, right: 0.21 },
          heading_deg: 90,
          sensor_distances_mm: { front: 100, left: null, right: 80 },
          robot_state: "NAVIGATING",
          error_reason: "",
        };
      },
    },
  };
  return { services: services as unknown as GatewayServices, calls };
}

function makeClient(url: string): Promise<Client> {
  return new Promise((resolve, reject) => {
    const socket = new WebSocket(url);
    const queue: Packet[] = [];
    const waiting: Array<(packet: Packet) => void> = [];
    socket.on("message", (data) => {
      let packet: Packet;
      try { packet = JSON.parse(data.toString()) as Packet; }
      catch { packet = { raw: data.toString() }; }
      const waiter = waiting.shift();
      if (waiter) waiter(packet); else queue.push(packet);
    });
    socket.once("open", () => resolve({
      socket,
      receive(timeoutMs = 1000) {
        const next = queue.shift();
        if (next) return Promise.resolve(next);
        return new Promise<Packet>((resolvePacket, rejectPacket) => {
          const timeout = setTimeout(() => {
            const index = waiting.indexOf(onPacket);
            if (index >= 0) waiting.splice(index, 1);
            rejectPacket(new Error("timeout aguardando mensagem WebSocket"));
          }, timeoutMs);
          const onPacket = (packet: Packet) => { clearTimeout(timeout); resolvePacket(packet); };
          waiting.push(onPacket);
        });
      },
    }));
    socket.once("error", reject);
  });
}

function send(client: Client, packet: unknown) {
  client.socket.send(typeof packet === "string" ? packet : JSON.stringify(packet));
}
async function expectNoMessage(client: Client) {
  await assert.rejects(client.receive(45), /timeout/);
}
async function hushErrors<T>(action: () => Promise<T>): Promise<T> {
  const old = console.error;
  console.error = () => {};
  try { return await action(); } finally { console.error = old; }
}

const contexts: Array<() => Promise<void>> = [];
afterEach(async () => {
  await Promise.all(contexts.splice(0).map((close) => close()));
});

async function setup(overrides: ServiceOverrides = {}) {
  const server = http.createServer();
  const { services, calls } = makeServices(overrides);
  const gateway = createTelemetryGateway(server, services);
  await new Promise<void>((resolve, reject) => {
    server.once("error", reject);
    server.listen(0, "127.0.0.1", () => { server.off("error", reject); resolve(); });
  });
  const address = server.address();
  if (!address || typeof address === "string") throw new Error("porta de teste indisponível");
  const base = `ws://127.0.0.1:${address.port}/ws`;
  const connected: Client[] = [];
  const connect = async (role?: "device" | "dashboard") => {
    const client = await makeClient(`${base}${role ? `?role=${role}` : ""}`);
    connected.push(client);
    return client;
  };
  contexts.push(async () => {
    for (const client of connected) {
      if (client.socket.readyState === WebSocket.OPEN || client.socket.readyState === WebSocket.CONNECTING) {
        await new Promise<void>((resolve) => {
          client.socket.once("close", () => resolve());
          client.socket.close();
          setTimeout(() => { if (client.socket.readyState !== WebSocket.CLOSED) client.socket.terminate(); }, 150).unref();
        });
      }
    }
    await new Promise<void>((resolve) => gateway.close(() => resolve()));
    if (server.listening) await new Promise<void>((resolve, reject) => server.close((err) => err ? reject(err) : resolve()));
  });
  return { connect, calls, gateway, server };
}

describe("telemetry.gateway WebSocket", () => {
  it("determina o papel da conexão e protege URL ausente ou malformada", () => {
    assert.equal(connectionRole({ url: "/ws?role=dashboard" } as IncomingMessage), "dashboard");
    assert.equal(connectionRole({ url: "/ws?role=device" } as IncomingMessage), "device");
    assert.equal(connectionRole({ url: undefined } as unknown as IncomingMessage), "device");
    assert.equal(connectionRole({ url: "http://[" } as IncomingMessage), "device");
  });

  it("envia somente quando o WebSocket está aberto", () => {
    const sent: string[] = [];
    const open = { readyState: WebSocket.OPEN, send: (data: string) => sent.push(data) } as unknown as WebSocket;
    const closed = { readyState: WebSocket.CLOSED, send: () => { throw new Error("não deve enviar"); } } as unknown as WebSocket;
    sendIfOpen(open, "{\"ok\":true}");
    sendIfOpen(closed, "{\"ignored\":true}");
    assert.deepEqual(sent, ['{"ok":true}']);
  });

  it("processa sessão, log, encerramento e telemetria e retransmite os resultados", async () => {
    const ctx = await setup();
    const dashboard = await ctx.connect("dashboard");
    const device = await ctx.connect("device");

    send(device, { type: "session_start", device_session_id: 77, tipo_labirinto: "4x4", tentativa: 2 });
    assert.deepEqual(await dashboard.receive(), { type: "session_started", device_session_id: 77, id_sessao: 31 });
    send(device, { type: "log", session_id: 77, nivel: "INFO", mensagem: "ok", t_ms: 55 });
    assert.deepEqual(await dashboard.receive(), { type: "event_recorded", session_id: 77, id_log: 10, timestamp_ms: 55, tipo_evento: "INFO", descricao: "ok" });
    send(device, { type: "session_end", session_id: 77, tempo_total_ms: 1000 });
    assert.deepEqual(await dashboard.receive(), { type: "session_finished", device_session_id: 77, id_sessao: 31, status_conclusao: "CONCLUIDO", pontos_registrados: 2 });
    send(device, { type: "telemetry", session_id: 77, t_ms: 250 });
    const telemetry = await dashboard.receive();
    assert.equal(telemetry.type, "telemetry");
    assert.deepEqual(telemetry.wheels_speed, { left: 0.2, right: 0.21 });
    assert.deepEqual(ctx.calls.map((call) => call.method), ["start", "log", "end", "telemetry"]);
  });

  it("devolve gateway_error para erros de domínio das três categorias", async () => {
    const ctx = await setup({
      startError: new SessionNavigationError("sessão inválida", 400),
      logError: new LogInvalidoError("log inválido"),
      telemetryError: new TelemetryInvalidaError("telemetria inválida"),
    });
    const device = await ctx.connect("device");
    send(device, { type: "session_start", device_session_id: 0 });
    assert.deepEqual(await device.receive(), { type: "gateway_error", message: "sessão inválida" });
    send(device, { type: "log", session_id: 0 });
    assert.deepEqual(await device.receive(), { type: "gateway_error", message: "log inválido" });
    send(device, { type: "telemetry", session_id: 0 });
    assert.deepEqual(await device.receive(), { type: "gateway_error", message: "telemetria inválida" });
  });

  it("descarta JSON malformado, arrays e erros inesperados sem encerrar a conexão", async () => {
    const ctx = await setup({ logError: new Error("erro de persistência") });
    const device = await ctx.connect("device");
    await hushErrors(async () => {
      send(device, "{json inválido");
      await new Promise((resolve) => setTimeout(resolve, 15));
      send(device, []);
      await new Promise((resolve) => setTimeout(resolve, 15));
      send(device, { type: "log", session_id: 1 });
      await new Promise((resolve) => setTimeout(resolve, 15));
    });
    await expectNoMessage(device);
    assert.equal(ctx.calls.length, 1);
    // Um pacote válido posterior comprova que o socket segue utilizável.
    send(device, { type: "unknown-kind" });
    await new Promise((resolve) => setTimeout(resolve, 20));
    assert.equal(device.socket.readyState, WebSocket.OPEN);
  });

  it("não permite que o dashboard grave no banco e ignora tipos desconhecidos", async () => {
    const ctx = await setup();
    const dashboard = await ctx.connect("dashboard");
    send(dashboard, { type: "session_start", device_session_id: 55 });
    await expectNoMessage(dashboard);
    const device = await ctx.connect("device");
    send(device, { type: "unknown-kind", arbitrary: true });
    await expectNoMessage(dashboard);
    assert.equal(ctx.calls.length, 0);
  });

  it("retransmite mensagens de mapa, cell e position sem modificar o formato", async () => {
    const ctx = await setup();
    const dashboard = await ctx.connect("dashboard");
    const device = await ctx.connect("device");
    const map = { width: 4, height: 4 };
    const cell = { type: "cell", cell: { x: 1, y: 2 }, robot: { x: 1, y: 2, heading: "NORTH" } };
    const position = { type: "position", x: 2, y: 2 };
    send(device, map); assert.deepEqual(await dashboard.receive(), map);
    send(device, cell); assert.deepEqual(await dashboard.receive(), cell);
    send(device, position); assert.deepEqual(await dashboard.receive(), position);
    send(device, { type: "cell", cell: { x: "bad", y: 2 }, robot: { x: 1 } });
    assert.deepEqual(await dashboard.receive(), { type: "cell", cell: { x: "bad", y: 2 }, robot: { x: 1 } });
    send(device, { type: "cell", cell: { x: 3, y: "bad" }, robot: { x: 1 } });
    assert.deepEqual(await dashboard.receive(), { type: "cell", cell: { x: 3, y: "bad" }, robot: { x: 1 } });
    send(device, { type: "position", robot: { x: 3 } });
    assert.deepEqual(await dashboard.receive(), { type: "position", robot: { x: 3 } });
  });

  it("envia snapshot ao dashboard novo e limpa dados antigos após novo maze", async () => {
    const ctx = await setup();
    const device = await ctx.connect("device");
    const maze1 = { type: "maze", width: 4, height: 4, id: 1 };
    const cell = { type: "cell", cell: { x: 1, y: 2 }, robot: { x: 1, y: 2, heading: "NORTH" } };
    send(device, maze1); send(device, cell); send(device, { type: "telemetry", session_id: 77 });
    await new Promise((resolve) => setTimeout(resolve, 30));
    const firstDashboard = await ctx.connect("dashboard");
    assert.deepEqual(await firstDashboard.receive(), maze1);
    const cachedCell = await firstDashboard.receive();
    assert.equal(cachedCell.type, "cell"); assert.deepEqual(cachedCell.robot, cell.robot);
    assert.equal((await firstDashboard.receive()).type, "telemetry");

    const maze2 = { type: "maze", width: 8, height: 4, id: 2 };
    send(device, maze2);
    await new Promise((resolve) => setTimeout(resolve, 15));
    const cellWithoutRobot = { type: "cell", cell: { x: 3, y: 0 }, wall: "NORTH" };
    send(device, cellWithoutRobot);
    await new Promise((resolve) => setTimeout(resolve, 15));
    const newDashboard = await ctx.connect("dashboard");
    assert.deepEqual(await newDashboard.receive(), maze2);
    assert.deepEqual(await newDashboard.receive(), cellWithoutRobot);
    await expectNoMessage(newDashboard);
  });

  it("aceita dashboard sem snapshot inicial sem enviar mensagens artificiais", async () => {
    const ctx = await setup();
    const dashboard = await ctx.connect("dashboard");
    await expectNoMessage(dashboard);
  });
});
