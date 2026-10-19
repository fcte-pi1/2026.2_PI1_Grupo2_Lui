import { WebSocketServer, WebSocket } from "ws";
import type { Server } from "node:http";
import { LogInvalidoError, type LogEventoService } from "../services/logEvento.service";

export function createTelemetryGateway(server: Server, logEventoService: LogEventoService) {
  const wss = new WebSocketServer({
    server,
    path: "/ws"
  });

  wss.on("connection", (socket: WebSocket) => {
    console.log("Dispositivo conectado via WebSocket");

    socket.on("message", async (data) => {
      try {
        const message: unknown = JSON.parse(data.toString());
        if (typeof message !== "object" || message === null || Array.isArray(message)) {
          console.error("Mensagem WebSocket deve ser um objeto JSON");
          return;
        }
        const packet = message as Record<string, unknown>;

        // HU-12: {"type":"log","session_id":1,"nivel":"WARN","mensagem":"...","t_ms":1200}
        if (packet.type === "log") {
          await logEventoService.registrarLog({
            idSessaoDispositivo: Number(packet.session_id),
            nivel: typeof packet.nivel === "string" ? packet.nivel : "",
            descricao: typeof packet.mensagem === "string" ? packet.mensagem : "",
            timestampMs: Number(packet.t_ms)
          });
          return;
        }

        console.log("Telemetria recebida:", packet);
      } catch (erro) {
        if (erro instanceof LogInvalidoError) {
          console.error("Log rejeitado:", erro.message);
          return;
        }
        console.error("Mensagem WebSocket inválida ou erro de persistência", erro);
      }
    });

    socket.on("close", () => {
      console.log("Dispositivo desconectado");
    });
  });

  return wss;
}