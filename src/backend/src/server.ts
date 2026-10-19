import http from "node:http";
import app from "./app";
import { createTelemetryGateway } from "./ws/telemetry.gateway";
import { logEventoService } from "./routes/logs.routes";

const PORT = Number(process.env.PORT) || 3000;

const server = http.createServer(app);

createTelemetryGateway(server, logEventoService);

server.listen(PORT, () => {
  console.log(`Backend executando em http://localhost:${PORT}`);
});