/** HU-09/HU-10: valida e persiste telemetria recebida do dispositivo. */
export interface RegistroTelemetriaRow {
  id_telemetria: bigint;
  id_sessao: number;
  timestamp_ms: bigint;
  tensao_bateria: number | null;
  corrente_bateria: number | null;
  distancia_frontal_mm: number | null;
  distancia_esquerda_mm: number | null;
  distancia_direita_mm: number | null;
  erro_pid: number;
  velocidade_esq: number;
  velocidade_dir: number;
  angulo_orientacao: number;
}

export interface TelemetryDb {
  sessaoNavegacao: {
    findUnique(args: { where: { id_dispositivo: bigint } }): Promise<{ id_sessao: number } | null>;
  };
  registroTelemetria: {
    create(args: { data: {
      id_sessao: number;
      timestamp_ms: bigint;
      tensao_bateria: number | null;
      corrente_bateria: number | null;
      distancia_frontal_mm: number | null;
      distancia_esquerda_mm: number | null;
      distancia_direita_mm: number | null;
      erro_pid: number;
      velocidade_esq: number;
      velocidade_dir: number;
      angulo_orientacao: number;
    } }): Promise<RegistroTelemetriaRow>;
  };
}

export interface TelemetriaValidada {
  session_id: number;
  timestamp_ms: number;
  battery_voltage: number | null;
  battery_current_a: number | null;
  pid_error: number;
  wheels_speed: { left: number; right: number };
  heading_deg: number;
  robot_state: string;
  sensor_distances_mm: { front: number | null; left: number | null; right: number | null };
  error_reason?: string;
}

export class TelemetryInvalidaError extends Error {}
const isRecord = (v: unknown): v is Record<string, unknown> =>
  typeof v === "object" && v !== null && !Array.isArray(v);
function finiteOrNull(v: unknown, field: string): number | null {
  if (v === null || v === undefined) return null;
  if (typeof v !== "number" || !Number.isFinite(v)) throw new TelemetryInvalidaError(`${field} inválido`);
  return v;
}
function finite(v: unknown, field: string): number {
  if (typeof v !== "number" || !Number.isFinite(v)) throw new TelemetryInvalidaError(`${field} inválido`);
  return v;
}

export function createTelemetryService(db: TelemetryDb) {
  return {
    async registrarTelemetria(raw: unknown): Promise<TelemetriaValidada> {
      if (!isRecord(raw)) throw new TelemetryInvalidaError("pacote de telemetria inválido");
      const sessionId = raw.session_id;
      if (!Number.isInteger(sessionId) || Number(sessionId) <= 0 || Number(sessionId) > 0xffffffff) {
        throw new TelemetryInvalidaError("session_id inválido");
      }
      const timestamp = finite(raw.t_ms ?? raw.timestamp, "timestamp");
      if (timestamp < 0 || timestamp > Number.MAX_SAFE_INTEGER) throw new TelemetryInvalidaError("timestamp fora do intervalo");
      const batteryVoltage = finiteOrNull(raw.battery_voltage, "battery_voltage");
      const batteryCurrent = finiteOrNull(raw.battery_current_a, "battery_current_a");
      const pidError = finite(raw.pid_error, "pid_error");
      if (!isRecord(raw.wheels_speed)) throw new TelemetryInvalidaError("wheels_speed inválido");
      const left = finite(raw.wheels_speed.left, "wheels_speed.left");
      const right = finite(raw.wheels_speed.right, "wheels_speed.right");
      const heading = finite(raw.heading_deg, "heading_deg");
      let sensorDistances = { front: null as number | null, left: null as number | null, right: null as number | null };
      if (raw.sensor_distances_mm !== undefined && raw.sensor_distances_mm !== null) {
        if (!isRecord(raw.sensor_distances_mm)) throw new TelemetryInvalidaError("sensor_distances_mm inválido");
        sensorDistances = {
          front: finiteOrNull(raw.sensor_distances_mm.front, "sensor_distances_mm.front"),
          left: finiteOrNull(raw.sensor_distances_mm.left, "sensor_distances_mm.left"),
          right: finiteOrNull(raw.sensor_distances_mm.right, "sensor_distances_mm.right"),
        };
        for (const distance of Object.values(sensorDistances)) {
          if (distance !== null && distance < 0) throw new TelemetryInvalidaError("distâncias de sensor não podem ser negativas");
        }
      }
      if (typeof raw.robot_state !== "string" || raw.robot_state.length > 64) {
        throw new TelemetryInvalidaError("robot_state inválido");
      }

      const session = await db.sessaoNavegacao.findUnique({ where: { id_dispositivo: BigInt(Number(sessionId)) } });
      if (!session) throw new TelemetryInvalidaError("sessão do dispositivo inexistente; envie session_start antes da telemetria");
      await db.registroTelemetria.create({ data: {
        id_sessao: session.id_sessao,
        timestamp_ms: BigInt(Math.trunc(timestamp)),
        tensao_bateria: batteryVoltage,
        corrente_bateria: batteryCurrent,
        distancia_frontal_mm: sensorDistances.front,
        distancia_esquerda_mm: sensorDistances.left,
        distancia_direita_mm: sensorDistances.right,
        erro_pid: pidError,
        velocidade_esq: left,
        velocidade_dir: right,
        angulo_orientacao: heading,
      } });

      return {
        session_id: Number(sessionId),
        timestamp_ms: timestamp,
        battery_voltage: batteryVoltage,
        battery_current_a: batteryCurrent,
        pid_error: pidError,
        wheels_speed: { left, right },
        heading_deg: heading,
        robot_state: raw.robot_state,
        sensor_distances_mm: sensorDistances,
        ...(typeof raw.error_reason === "string" ? { error_reason: raw.error_reason.slice(0, 240) } : {}),
      };
    },
  };
}
export type TelemetryService = ReturnType<typeof createTelemetryService>;
