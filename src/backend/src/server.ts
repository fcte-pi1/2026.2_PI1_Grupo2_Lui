import http, { type Server } from "node:http";
import type { Express } from "express";
import { createApp } from "./app";
import { createLogsRouter } from "./routes/logs.routes";
import { createTelemetryGateway, ingestBluetoothMessage } from "./ws/telemetry.gateway";
import { startBluetoothSerialBridge, type BluetoothSerialBridge } from "./communication/bluetoothSerial";
import { createLogEventoService, type LogEventoDb } from "./services/logEvento.service";
import { createSessionListService, type SessionListDb } from "./services/sessionList.service";
import { createSessionNavigationService, type SessionNavigationDb } from "./services/sessionNavigation.service";
import { createTelemetryService, type TelemetryDb } from "./services/telemetry.service";
import { createPrismaClient } from "./lib/prisma";
import type { WebSocketServer } from "ws";

export interface BackendServices {
  logs: ReturnType<typeof createLogEventoService>;
  sessions: ReturnType<typeof createSessionNavigationService>;
  telemetry: ReturnType<typeof createTelemetryService>;
}

export interface BackendServer {
  server: Server;
  gateway: WebSocketServer;
  bluetoothBridge?: BluetoothSerialBridge;
}

export interface StartServerOptions {
  port?: number;
  /** Ponto de injeção para testes; em produção usa createPrismaClient. */
  databaseFactory?: () => Promise<unknown>;
  log?: (message: string) => void;
}

export function resolvePort(value: string | undefined = process.env.PORT): number {
  return Number(value) || 3000;
}

/** Compõe HTTP + WebSocket sem abrir a porta; útil para testes de integração. */
export function createBackendServer(services: BackendServices, app: Express): BackendServer {
  const server = http.createServer(app);
  const gateway = createTelemetryGateway(server, services);
  return { server, gateway };
}

/** Inicializa o banco, compõe os serviços e só então começa a escutar. */
export async function startServer(options: StartServerOptions = {}): Promise<BackendServer> {
  const port = options.port ?? resolvePort();
  const databaseFactory = options.databaseFactory ?? createPrismaClient;
  const log = options.log ?? console.log;
  const db = await databaseFactory();

  const logs = createLogEventoService(db as LogEventoDb);
  const sessoes = createSessionListService(db as SessionListDb);
  const sessions = createSessionNavigationService(db as SessionNavigationDb);
  const telemetry = createTelemetryService(db as TelemetryDb);
  const router = createLogsRouter({ logs, sessoes, navegacao: sessions });
  const backend = createBackendServer({ logs, sessions, telemetry }, createApp(router));

  await new Promise<void>((resolveListen, rejectListen) => {
    backend.server.once("error", rejectListen);
    backend.server.listen(port, () => {
      backend.server.off("error", rejectListen);
      log(`Backend executando em http://localhost:${port}`);
      resolveListen();
    });
  });

  // A porta serial virtual é criada pelo sistema operacional após o pareamento
  // Bluetooth SPP. Sem a variável, o backend continua operando sem hardware.
  const bluetoothPath = process.env.MM_BLUETOOTH_SERIAL_PORT?.trim();
  if (bluetoothPath) {
    const baudRate = Number(process.env.MM_BLUETOOTH_SERIAL_BAUD_RATE) || 115200;
    backend.bluetoothBridge = startBluetoothSerialBridge({
      path: bluetoothPath,
      baudRate,
      onLine: (line) => ingestBluetoothMessage(backend.gateway, line),
      log,
    });
    log(`Ponte Bluetooth SPP habilitada em ${bluetoothPath} (baud=${baudRate}).`);
  } else {
    log("Ponte Bluetooth SPP desabilitada; defina MM_BLUETOOTH_SERIAL_PORT após parear a ESP32.");
  }

  return backend;
}

