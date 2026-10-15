import http from "node:http";
import app from "./app";
import { createTelemetryGateway } from "./ws/telemetry.gateway";

const PORT = Number(process.env.PORT) || 3000;

const server = http.createServer(app);

createTelemetryGateway(server);

server.listen(PORT, () => {
  console.log(`Backend executando em http://localhost:${PORT}`);
});