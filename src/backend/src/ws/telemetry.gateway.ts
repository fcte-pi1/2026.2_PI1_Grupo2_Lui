import { WebSocketServer, WebSocket } from "ws";
import type { IncomingMessage, Server } from "node:http";
import { LogInvalidoError, type LogEventoService } from "../services/logEvento.service";
import { SessionNavigationError, type SessionNavigationService } from "../services/sessionNavigation.service";
import { TelemetryInvalidaError, type TelemetryService } from "../services/telemetry.service";

type GatewayServices = {
  logs: LogEventoService;
  sessions: SessionNavigationService;
  telemetry: TelemetryService;
};
type Packet = Record<string, unknown>;

/** Evita enviar em sockets fechados ou em encerramento. */
export function sendIfOpen(socket: WebSocket, serialized: string): void {
  if (socket.readyState === WebSocket.OPEN) socket.send(serialized);
}

function isPacket(value: unknown): value is Packet {
  return typeof value === "object" && value !== null && !Array.isArray(value);
}
export function connectionRole(request: Pick<IncomingMessage, "url">): "device" | "dashboard" {
  try {
    const url = new URL(request.url ?? "/ws", "http://localhost");
    return url.searchParams.get("role") === "dashboard" ? "dashboard" : "device";
  } catch {
    return "device";
  }
}

/** HU-10: recebe do dispositivo, persiste os dados de domínio e retransmite
 * apenas mensagens permitidas aos navegadores conectados. */
export function createTelemetryGateway(server: Server, services: GatewayServices) {
  const wss = new WebSocketServer({ server, path: "/ws" });
  const dashboards = new Set<WebSocket>();
  let latestMaze: Packet | null = null;
  let latestTelemetry: Packet | null = null;
  let latestRobot: Packet | null = null;
  const latestCells = new Map<string, Packet>();

  const broadcast = (packet: Packet) => {
    if (packet.type === "maze") {
      // Uma nova configuração inicia outra corrida: nunca misturar células ou
      // telemetria da sessão anterior no snapshot servido a um dashboard novo.
      latestMaze = packet;
      latestCells.clear();
      latestRobot = null;
      latestTelemetry = null;
    }
    if (packet.type === "telemetry") latestTelemetry = packet;
    if (isPacket(packet.cell)) {
      // Atualizações de célula podem chegar sem o bloco de posição do robô.
      // O snapshot ainda deve conservar a célula e incluir o robô apenas quando conhecido.
      if (isPacket(packet.robot)) latestRobot = packet.robot;
      const x = packet.cell.x;
      const y = packet.cell.y;
      if (typeof x === "number" && typeof y === "number") latestCells.set(`${x},${y}`, packet);
    }
    const serialized = JSON.stringify(packet);
    for (const client of dashboards) sendIfOpen(client, serialized!);
  };

  wss.on("connection", (socket: WebSocket, request: IncomingMessage) => {
    const role = connectionRole(request);
    if (role === "dashboard") {
      dashboards.add(socket);
      console.log("Dashboard conectado via WebSocket");
    } else {
      console.log("Dispositivo conectado via WebSocket");
    }
    if (role === "dashboard") {
      // Reconexões/recarregamentos da página recebem um snapshot do último mapa
      // e da última telemetria sem exigir reinício da missão no ESP32.
      if (latestMaze) socket.send(JSON.stringify(latestMaze));
      for (const saved of latestCells.values()) {
        const cellOnly = { ...saved, ...(latestRobot ? { robot: latestRobot } : {}) };
        socket.send(JSON.stringify(cellOnly));
      }
      if (latestTelemetry) socket.send(JSON.stringify(latestTelemetry));
    }

    // Serializa o processamento por conexão: session_start deve ser persistido
    // antes dos logs/telemetria e session_end, mesmo com consultas assíncronas.
    let processing = Promise.resolve();
    socket.on("message", (data) => {
      processing = processing.then(async () => {
        if (role !== "device") return; // navegador não grava no banco pelo WS.
        try {
          const parsed: unknown = JSON.parse(data.toString());
          if (!isPacket(parsed)) throw new Error("Mensagem WebSocket deve ser um objeto JSON");
          const packet = parsed;

          if (packet.type === "session_start") {
            const created = await services.sessions.iniciarSessao({
              device_session_id: Number(packet.device_session_id),
              tipo_labirinto: typeof packet.tipo_labirinto === "string" ? packet.tipo_labirinto : undefined,
              tentativa: packet.tentativa === undefined ? undefined : Number(packet.tentativa),
            });
            broadcast({ type: "session_started", device_session_id: Number(packet.device_session_id), id_sessao: created.id_sessao });
            return;
          }

          if (packet.type === "log") {
            const event = await services.logs.registrarLog({
              idSessaoDispositivo: Number(packet.session_id),
              nivel: typeof packet.nivel === "string" ? packet.nivel : "",
              descricao: typeof packet.mensagem === "string" ? packet.mensagem : "",
              timestampMs: Number(packet.t_ms),
            });
            broadcast({ type: "event_recorded", session_id: Number(packet.session_id), ...event });
            return;
          }

          if (packet.type === "session_end") {
            const result = await services.sessions.finalizarSessao(packet);
            broadcast({
              type: "session_finished",
              device_session_id: Number(packet.session_id),
              ...result,
            });
            return;
          }

          if (packet.type === "telemetry") {
            const telemetry = await services.telemetry.registrarTelemetria(packet);
            broadcast({
              type: "telemetry",
              battery_voltage: telemetry.battery_voltage,
              battery_current_a: telemetry.battery_current_a,
              pid_error: telemetry.pid_error,
              wheels_speed: telemetry.wheels_speed,
              heading_deg: telemetry.heading_deg,
              sensor_distances_mm: telemetry.sensor_distances_mm,
              robot_state: telemetry.robot_state,
              error_reason: telemetry.error_reason,
              timestamp: telemetry.timestamp_ms,
            });
            return;
          }

          // Pacote de mapa do MazeMap (sem `type`) e mensagens tipadas maze,
          // cell ou position são encaminhados sem criar uma segunda tradução.
          if (!packet.type || ["maze", "cell", "position"].includes(String(packet.type))) {
            broadcast(packet);
          }
        } catch (error) {
          if (error instanceof LogInvalidoError || error instanceof SessionNavigationError || error instanceof TelemetryInvalidaError) {
            console.error("Pacote de telemetria rejeitado:", error.message);
            sendIfOpen(socket, JSON.stringify({ type: "gateway_error", message: error.message })!);
            return;
          }
          console.error("Mensagem WebSocket inválida ou erro de persistência", error);
        }
      }).catch((error) => console.error("Falha no processamento serial do WebSocket", error));
    });

    socket.on("close", () => {
      dashboards.delete(socket);
      console.log(role === "dashboard" ? "Dashboard desconectado" : "Dispositivo desconectado");
    });
  });

  wss.on("close", () => {
    dashboards.clear();
    latestCells.clear();
  });
  return wss;
}
