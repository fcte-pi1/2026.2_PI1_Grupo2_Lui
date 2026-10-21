import http from "node:http";
import app from "./app";
import { createTelemetryGateway } from "./ws/telemetry.gateway";
import { logEventoService, sessionNavigationService, telemetryPersistenceService } from "./routes/logs.routes";

const PORT = Number(process.env.PORT) || 3000;

const server = http.createServer(app);

createTelemetryGateway(server, { logs: logEventoService, sessions: sessionNavigationService, telemetry: telemetryPersistenceService });

server.listen(PORT, () => {
  console.log(`Backend executando em http://localhost:${PORT}`);
});