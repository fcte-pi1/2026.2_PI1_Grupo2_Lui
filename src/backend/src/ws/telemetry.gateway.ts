import { WebSocketServer, WebSocket } from "ws";
import type { Server } from "node:http";
import { logEventoService } from "../routes/logs.routes";
import { LogInvalidoError } from "../services/logEvento.service";

export function createTelemetryGateway(server: Server) {
  const wss = new WebSocketServer({
    server,
    path: "/ws"
  });

  wss.on("connection", (socket: WebSocket) => {
    console.log("Dispositivo conectado via WebSocket");

    socket.on("message", async (data) => {
      try {
        const message = JSON.parse(data.toString());

        // HU-12: {"type":"log","session_id":1,"nivel":"WARN","mensagem":"...","t_ms":1200}
        if (message?.type === "log") {
          await logEventoService.registrarLog({
            idSessao: Number(message.session_id),
            nivel: message.nivel,
            descricao: message.mensagem,
            timestampMs: Number(message.t_ms)
          });
          return;
        }

        console.log("Telemetria recebida:", message);

        // Futuramente:
        // 1. validar JSON
        // 2. identificar sessão
        // 3. persistir no PostgreSQL
        // 4. retransmitir ao Dashboard
      } catch (erro) {
        if (erro instanceof LogInvalidoError) {
          console.error("Log rejeitado:", erro.message);
          return;
        }
        console.error("Mensagem WebSocket inválida");
      }
    });

    socket.on("close", () => {
      console.log("Dispositivo desconectado");
    });
  });

  return wss;
}