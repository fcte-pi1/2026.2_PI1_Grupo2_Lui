import { describe, it } from "node:test";
import assert from "node:assert/strict";
import {
  createTelemetryService,
  TelemetryInvalidaError,
  type TelemetryDb,
} from "../telemetry.service";

function criarDbFake() {
  const inserted: Array<Record<string, unknown>> = [];
  const db: TelemetryDb = {
    sessaoNavegacao: {
      async findUnique({ where }) {
        return where.id_dispositivo === 77n ? { id_sessao: 5 } : null;
      },
    },
    registroTelemetria: {
      async create({ data }) {
        inserted.push(data as unknown as Record<string, unknown>);
        return { id_telemetria: BigInt(inserted.length), ...data };
      },
    },
  };
  return { db, inserted };
}

const payload = {
  type: "telemetry",
  session_id: 77,
  t_ms: 1250,
  timestamp: 1250,
  battery_voltage: null,
  battery_current_a: null,
  pid_error: -0.12,
  wheels_speed: { left: 0.2, right: 0.19 },
  heading_deg: 90,
  robot_state: "NAVIGATING",
  sensor_distances_mm: { front: 120, left: null, right: 80 },
};

describe("HU-09/HU-10 telemetry persistence", () => {
  it("converte o ID do dispositivo para a sessão do banco e permite bateria indisponível", async () => {
    const { db, inserted } = criarDbFake();
    const result = await createTelemetryService(db).registrarTelemetria(payload);
    assert.equal(inserted.length, 1);
    assert.equal(inserted[0].id_sessao, 5);
    assert.equal(inserted[0].timestamp_ms, 1250n);
    assert.equal(inserted[0].tensao_bateria, null);
    assert.equal(inserted[0].corrente_bateria, null);
    assert.equal(inserted[0].distancia_frontal_mm, 120);
    assert.equal(inserted[0].distancia_esquerda_mm, null);
    assert.equal(inserted[0].erro_pid, -0.12);
    assert.equal(result.sensor_distances_mm.right, 80);
    assert.equal(result.wheels_speed.left, 0.2);
    assert.equal(result.heading_deg, 90);
  });

  it("rejeita pacote inválido e sessão de dispositivo não registrada", async () => {
    const { db } = criarDbFake();
    const service = createTelemetryService(db);
    await assert.rejects(() => service.registrarTelemetria({ ...payload, pid_error: Number.NaN }), TelemetryInvalidaError);
    await assert.rejects(() => service.registrarTelemetria({ ...payload, session_id: 78 }), TelemetryInvalidaError);
  });
});
