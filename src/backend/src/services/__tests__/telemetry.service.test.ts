import { describe, it } from "node:test";
import assert from "node:assert/strict";
import {
  createTelemetryService,
  TelemetryInvalidaError,
  type TelemetryDb,
} from "../telemetry.service";

function criarDbFake(opcoes: { sessao?: { id_sessao: number } | null } = {}) {
  const inserted: Array<Record<string, unknown>> = [];
  const consultas: bigint[] = [];
  const db: TelemetryDb = {
    sessaoNavegacao: {
      async findUnique({ where }) {
        consultas.push(where.id_dispositivo);
        if (Object.hasOwn(opcoes, "sessao")) return opcoes.sessao ?? null;
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
  return { db, inserted, consultas };
}

const payload = {
  type: "telemetry",
  session_id: 77,
  t_ms: 1250,
  timestamp: 1300,
  battery_voltage: null as number | null,
  battery_current_a: null as number | null,
  pid_error: -0.12,
  wheels_speed: { left: 0.2, right: 0.19 },
  heading_deg: 90,
  robot_state: "NAVIGATING",
  sensor_distances_mm: { front: 120, left: null as number | null, right: 80 },
};

async function rejeita(raw: unknown, mensagem?: RegExp) {
  await assert.rejects(
    () => createTelemetryService(criarDbFake().db).registrarTelemetria(raw),
    (error: unknown) => error instanceof TelemetryInvalidaError && (!mensagem || mensagem.test(error.message)),
  );
}

describe("telemetry.service — persistência válida", () => {
  it("mapeia o ID do dispositivo para a sessão do banco e permite bateria indisponível", async () => {
    const { db, inserted, consultas } = criarDbFake();
    const result = await createTelemetryService(db).registrarTelemetria(payload);
    assert.equal(inserted.length, 1);
    assert.deepEqual(consultas, [77n]);
    assert.equal(inserted[0].id_sessao, 5);
    assert.equal(inserted[0].timestamp_ms, 1250n);
    assert.equal(inserted[0].tensao_bateria, null);
    assert.equal(inserted[0].corrente_bateria, null);
    assert.equal(inserted[0].distancia_frontal_mm, 120);
    assert.equal(inserted[0].distancia_esquerda_mm, null);
    assert.equal(inserted[0].distancia_direita_mm, 80);
    assert.equal(inserted[0].erro_pid, -0.12);
    assert.equal(inserted[0].velocidade_esq, 0.2);
    assert.equal(inserted[0].velocidade_dir, 0.19);
    assert.equal(inserted[0].angulo_orientacao, 90);
    assert.equal(result.sensor_distances_mm.right, 80);
    assert.equal(result.wheels_speed.left, 0.2);
    assert.equal(result.heading_deg, 90);
    assert.equal(result.timestamp_ms, 1250);
    assert.equal("error_reason" in result, false);
  });

  it("aceita timestamp alternativo, valores numéricos da bateria e distâncias ausentes", async () => {
    const { db, inserted } = criarDbFake();
    const raw = {
      ...payload,
      t_ms: undefined,
      timestamp: 1999.75,
      battery_voltage: 7.4,
      battery_current_a: 0.32,
      sensor_distances_mm: { front: undefined, left: 0, right: null },
      error_reason: "Falha simulada".repeat(40),
    };
    const result = await createTelemetryService(db).registrarTelemetria(raw);
    assert.equal(result.timestamp_ms, 1999.75);
    assert.equal(inserted[0].timestamp_ms, 1999n);
    assert.equal(inserted[0].tensao_bateria, 7.4);
    assert.equal(inserted[0].corrente_bateria, 0.32);
    assert.deepEqual(result.sensor_distances_mm, { front: null, left: 0, right: null });
    assert.equal(result.error_reason?.length, 240);
  });

  it("aceita sensor_distances_mm null/ausente e ignora error_reason não textual", async () => {
    const service = createTelemetryService(criarDbFake().db);
    const semSensor = { ...payload, sensor_distances_mm: undefined, error_reason: 123 };
    const a = await service.registrarTelemetria(semSensor);
    assert.deepEqual(a.sensor_distances_mm, { front: null, left: null, right: null });
    assert.equal("error_reason" in a, false);
    const sensorNulo = await service.registrarTelemetria({ ...payload, sensor_distances_mm: null });
    assert.deepEqual(sensorNulo.sensor_distances_mm, { front: null, left: null, right: null });
  });

  it("registra o estado e aceita timestamp igual ao limite seguro", async () => {
    const result = await createTelemetryService(criarDbFake().db).registrarTelemetria({
      ...payload,
      t_ms: Number.MAX_SAFE_INTEGER,
      robot_state: "FINISHED",
      error_reason: "",
    });
    assert.equal(result.timestamp_ms, Number.MAX_SAFE_INTEGER);
    assert.equal(result.robot_state, "FINISHED");
    assert.equal(result.error_reason, "");
  });
});

describe("telemetry.service — validação do pacote", () => {
  it("rejeita valores que não são objetos JSON", async () => {
    for (const raw of [null, undefined, "telemetry", 12, true, [], [payload]]) {
      await rejeita(raw, /pacote de telemetria inválido/);
    }
  });

  it("valida session_id como inteiro positivo de 32 bits", async () => {
    for (const session_id of [undefined, null, "77", 1.5, 0, -1, 0x1_0000_0000]) {
      await rejeita({ ...payload, session_id }, /session_id inválido/);
    }
  });

  it("valida timestamp requerido, tipo numérico e faixa permitida", async () => {
    await rejeita({ ...payload, t_ms: undefined, timestamp: undefined }, /timestamp inválido/);
    for (const t_ms of ["12", Number.NaN, Number.POSITIVE_INFINITY, Number.NEGATIVE_INFINITY]) {
      await rejeita({ ...payload, t_ms }, /timestamp inválido/);
    }
    await rejeita({ ...payload, t_ms: -1 }, /timestamp fora do intervalo/);
    await rejeita({ ...payload, t_ms: Number.MAX_SAFE_INTEGER + 1 }, /timestamp fora do intervalo/);
  });

  it("valida tensão e corrente — null/undefined são permitidos, outros valores precisam ser finitos", async () => {
    for (const field of ["battery_voltage", "battery_current_a"] as const) {
      for (const invalid of ["7.2", Number.NaN, Number.POSITIVE_INFINITY]) {
        await rejeita({ ...payload, [field]: invalid }, new RegExp(`${field} inválido`));
      }
    }
    const result = await createTelemetryService(criarDbFake().db).registrarTelemetria({
      ...payload,
      battery_voltage: undefined,
      battery_current_a: null,
    });
    assert.equal(result.battery_voltage, null);
    assert.equal(result.battery_current_a, null);
  });

  it("valida pid_error, wheels_speed, heading e cada velocidade", async () => {
    for (const pid_error of [undefined, "0", Number.NaN, Number.POSITIVE_INFINITY]) {
      await rejeita({ ...payload, pid_error }, /pid_error inválido/);
    }
    for (const wheels_speed of [null, [], "0", undefined]) {
      await rejeita({ ...payload, wheels_speed }, /wheels_speed inválido/);
    }
    await rejeita({ ...payload, wheels_speed: { left: "0", right: 1 } }, /wheels_speed.left inválido/);
    await rejeita({ ...payload, wheels_speed: { left: 1, right: Number.NaN } }, /wheels_speed.right inválido/);
    for (const heading_deg of [undefined, "90", Number.NaN]) {
      await rejeita({ ...payload, heading_deg }, /heading_deg inválido/);
    }
  });

  it("valida a estrutura e a não negatividade das distâncias", async () => {
    for (const sensor_distances_mm of [false, 0, "120", []]) {
      await rejeita({ ...payload, sensor_distances_mm }, /sensor_distances_mm inválido/);
    }
    for (const [field, value] of [["front", -1], ["left", -0.1], ["right", -50]] as const) {
      await rejeita({ ...payload, sensor_distances_mm: { front: 1, left: 2, right: 3, [field]: value } }, /distâncias de sensor não podem ser negativas/);
    }
    await rejeita({ ...payload, sensor_distances_mm: { front: "120", left: null, right: 80 } }, /sensor_distances_mm.front inválido/);
    await rejeita({ ...payload, sensor_distances_mm: { front: 120, left: Number.POSITIVE_INFINITY, right: 80 } }, /sensor_distances_mm.left inválido/);
  });

  it("exige robot_state textual com até 64 caracteres", async () => {
    for (const robot_state of [undefined, null, 12, {}, "x".repeat(65)]) {
      await rejeita({ ...payload, robot_state }, /robot_state inválido/);
    }
  });

  it("rejeita distâncias ou sessão não registrada e não persiste amostras inválidas", async () => {
    const { db, inserted } = criarDbFake({ sessao: null });
    await assert.rejects(
      () => createTelemetryService(db).registrarTelemetria(payload),
      (error: unknown) => error instanceof TelemetryInvalidaError && /sessão do dispositivo inexistente/.test(error.message),
    );
    assert.equal(inserted.length, 0);
  });
});
