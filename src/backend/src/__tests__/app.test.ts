import { afterEach, describe, it } from "node:test";
import assert from "node:assert/strict";
import type { Server } from "node:http";
import express from "express";
import { createApp } from "../app";

const activeServers: Server[] = [];
afterEach(async () => {
  await Promise.all(activeServers.splice(0).map(async (server) => {
    if (server.listening) await new Promise<void>((resolve, reject) => server.close((error) => error ? reject(error) : resolve()));
  }));
});
async function start(app: ReturnType<typeof createApp>) {
  const server = app.listen(0, "127.0.0.1"); activeServers.push(server);
  await new Promise<void>((resolve) => server.once("listening", resolve));
  const address = server.address();
  if (!address || typeof address === "string") throw new Error("endereço de teste indisponível");
  return `http://127.0.0.1:${address.port}`;
}

describe("app", () => {
  it("expõe o health check com o contrato esperado", async () => {
    const base = await start(createApp());
    const response = await fetch(`${base}/health`);
    assert.equal(response.status, 200);
    assert.deepEqual(await response.json(), { status: "ok", service: "micromouse-backend" });
  });

  it("responde preflight CORS", async () => {
    const base = await start(createApp());
    const response = await fetch(`${base}/health`, { method: "OPTIONS", headers: { origin: "http://localhost:5173", "access-control-request-method": "GET" } });
    assert.equal(response.status, 204);
    assert.equal(response.headers.get("access-control-allow-origin"), "*");
  });

  it("monta o roteador opcional sob /api/sessions", async () => {
    const router = express.Router();
    router.get("/probe", (_req, res) => res.json({ reached: true }));
    const base = await start(createApp(router));
    const response = await fetch(`${base}/api/sessions/probe`);
    assert.deepEqual(await response.json(), { reached: true });
  });

  it("não monta endpoints de sessão quando não é fornecido roteador", async () => {
    const base = await start(createApp());
    assert.equal((await fetch(`${base}/api/sessions/probe`)).status, 404);
  });

  it("ativa o parser JSON antes do roteador", async () => {
    const router = express.Router();
    router.post("/echo", (req, res) => res.json({ body: req.body }));
    const base = await start(createApp(router));
    const response = await fetch(`${base}/api/sessions/echo`, { method: "POST", headers: { "content-type": "application/json" }, body: JSON.stringify({ hello: "world" }) });
    assert.deepEqual(await response.json(), { body: { hello: "world" } });
  });
});
