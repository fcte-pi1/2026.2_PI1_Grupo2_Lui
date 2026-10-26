import { describe, it } from "node:test";
import assert from "node:assert/strict";
import net from "node:net";
import { resolvePort, startServer } from "../server";


async function getFreePort() {
  const probe = net.createServer();
  await new Promise<void>((resolve, reject) => probe.listen(0, "127.0.0.1", () => resolve()));
  const address = probe.address();
  if (!address || typeof address === "string") throw new Error("porta temporária indisponível");
  await new Promise<void>((resolve, reject) => probe.close((error) => error ? reject(error) : resolve()));
  return address.port;
}

async function closeBackend(backend: { server: import("node:http").Server; gateway: import("ws").WebSocketServer }) {
  await new Promise<void>((resolveClose) => backend.gateway.close(() => resolveClose()));
  if (backend.server.listening) {
    await new Promise<void>((resolveClose, rejectClose) => backend.server.close((error) => error ? rejectClose(error) : resolveClose()));
  }
}

describe("server startup", () => {
  it("resolvePort seleciona porta configurada ou padrão", () => {
    assert.equal(resolvePort("4567"), 4567);
    assert.equal(resolvePort("not-a-port"), 3000);
    assert.equal(resolvePort("0"), 3000);
    assert.equal(resolvePort(""), 3000);
  });

  it("inicializa HTTP e WebSocket com uma fábrica de banco substituível", async () => {
    const logged: string[] = [];
    const backend = await startServer({ port: 0, databaseFactory: async () => ({}), log: (message) => logged.push(message) });
    try {
      assert.equal(backend.server.listening, true);
      assert.equal(backend.gateway.clients.size, 0);
      assert.equal(logged.length, 1);
      const address = backend.server.address();
      assert.ok(address && typeof address !== "string");
      const response = await fetch(`http://127.0.0.1:${address.port}/health`);
      assert.equal(response.status, 200);
      assert.equal((await response.json() as any).service, "micromouse-backend");
    } finally {
      await closeBackend(backend);
    }
  });

  it("propaga falha ao criar a conexão de banco", async () => {
    await assert.rejects(
      () => startServer({ port: 0, databaseFactory: async () => { throw new Error("db não disponível"); }, log: () => {} }),
      /db não disponível/,
    );
  });

  it("usa PORT do ambiente quando a opção port não é fornecida", async () => {
    const previousPort = process.env.PORT;
    const port = await getFreePort();
    process.env.PORT = String(port);
    const oldLog = console.log;
    console.log = () => {};
    let backend: Awaited<ReturnType<typeof startServer>> | undefined;
    try {
      backend = await startServer({ databaseFactory: async () => ({}) });
      const address = backend.server.address();
      assert.ok(address && typeof address !== "string");
      assert.equal(address.port, port);
    } finally {
      console.log = oldLog;
      if (previousPort === undefined) delete process.env.PORT; else process.env.PORT = previousPort;
      if (backend) await closeBackend(backend);
    }
  });

  it("usa a fábrica Prisma padrão quando não é injetada", async () => {
    const previousUrl = process.env.DATABASE_URL;
    delete process.env.DATABASE_URL;
    try {
      await assert.rejects(() => startServer({ port: 0, log: () => {} }), /DATABASE_URL não configurada/);
    } finally {
      if (previousUrl === undefined) delete process.env.DATABASE_URL; else process.env.DATABASE_URL = previousUrl;
    }
  });
});
