/** Conexão dashboard -> gateway HU-10. URL substituível no .env via VITE_WS_URL. */
const WS_URL = import.meta.env.VITE_WS_URL ?? "ws://localhost:3000/ws?role=dashboard";

export function connectTelemetry(onMessage: (data: unknown) => void) {
  const socket = new WebSocket(WS_URL);
  socket.onopen = () => console.log("WebSocket do dashboard conectado");
  socket.onmessage = (event) => {
    try {
      onMessage(JSON.parse(event.data) as unknown);
    } catch {
      console.error("Mensagem WebSocket inválida; JSON descartado");
    }
  };
  socket.onerror = () => console.error("Falha na conexão WebSocket do dashboard");
  socket.onclose = () => console.log("WebSocket do dashboard encerrado");
  return socket;
}
