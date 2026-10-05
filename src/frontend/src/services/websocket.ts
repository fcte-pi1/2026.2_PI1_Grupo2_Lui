export function connectTelemetry(
  onMessage: (data: unknown) => void
) {
  const socket = new WebSocket("ws://localhost:3000/ws");

  socket.onopen = () => {
    console.log("WebSocket conectado");
  };

  socket.onmessage = (event) => {
    try {
      const data = JSON.parse(event.data);
      onMessage(data);
    } catch {
      console.error("Mensagem inválida");
    }
  };

  socket.onclose = () => {
    console.log("WebSocket encerrado");
  };

  return socket;
}