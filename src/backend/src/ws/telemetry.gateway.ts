import { WebSocketServer, WebSocket } from "ws";
import type { Server } from "node:http";

export function createTelemetryGateway(server: Server) {
  const wss = new WebSocketServer({
    server,
    path: "/ws"
  });

  wss.on("connection", (socket: WebSocket) => {
    console.log("Dispositivo conectado via WebSocket");

    socket.on("message", (data) => {
      try {
        const message = JSON.parse(data.toString());

        console.log("Telemetria recebida:", message);

        // Futuramente:
        // 1. validar JSON
        // 2. identificar sessão
        // 3. persistir no PostgreSQL
        // 4. retransmitir ao Dashboard
      } catch {
        console.error("Mensagem WebSocket inválida");
      }
    });

    socket.on("close", () => {
      console.log("Dispositivo desconectado");
    });
  });

  return wss;
}