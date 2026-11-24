import { createRobotControl, createRobotControlRouter, type RobotControl } from "./communication/robotControl";
import http, { type Server } from "node:http";
import { Router } from "express";
import { startBluetoothNativeBridge } from "./communication/bluetoothNative";
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
  robotControl?: RobotControl;
  closeDatabase?: () => Promise<void>;
}

export interface StartServerOptions {
  port?: number;
  liveOnly?: boolean;
  /** Ponto de injeção para testes; em produção usa createPrismaClient. */
  databaseFactory?: () => Promise<unknown>;
  log?: (message: string) => void;
}

export function resolvePort(value: string | undefined = process.env.PORT): number {
  return Number(value) || 3000;
}

/** Compõe HTTP + WebSocket sem abrir a porta; útil para testes de integração. */
export function createBackendServer(services: BackendServices | undefined, app: Express): BackendServer {
  const server = http.createServer(app);
  const gateway = createTelemetryGateway(server, services);
  const backend: BackendServer = {server,gateway};
  const control=createRobotControl({send:line=>backend.bluetoothBridge?.send?.(line) ?? false});
  backend.robotControl=control;
  app.use("/api/robot",createRobotControlRouter(control));
  gateway.on("robot_control_packet",(line:string)=>control.observe(line));
  return backend;
}

/** Inicializa o banco, compõe os serviços e só então começa a escutar. */
export async function startServer(options: StartServerOptions = {}): Promise<BackendServer> {
  const port = options.port ?? resolvePort();
  const liveOnly=options.liveOnly ?? process.env.MM_DIAGNOSTIC_ONLY === "1";
  const log=options.log ?? console.log;
  let backend: BackendServer;
  if (liveOnly) {
    const router=Router();
    router.use((_req,res)=>res.status(503).json({erro:"Monitor ao vivo: nenhuma corrida está sendo registrada"}));
    backend=createBackendServer(undefined,createApp(router));
    log("Modo monitor ao vivo; histórico e persistência de corridas desabilitados.");
  } else {
    const db=await (options.databaseFactory ?? createPrismaClient)();
    const logs=createLogEventoService(db as LogEventoDb);
    const sessoes=createSessionListService(db as SessionListDb);
    const sessions=createSessionNavigationService(db as SessionNavigationDb);
    const telemetry=createTelemetryService(db as TelemetryDb);
    const router=createLogsRouter({logs,sessoes,navegacao:sessions});
    backend=createBackendServer({logs,sessions,telemetry},createApp(router));
    if(db && typeof db === "object" && "$disconnect" in db && typeof db.$disconnect === "function")
      backend.closeDatabase=()=>db.$disconnect();
  }

  await new Promise<void>((resolveListen, rejectListen) => {
    backend.server.once("error", rejectListen);
    backend.server.listen({port,...(liveOnly?{host:"127.0.0.1"}:{})}, () => {
      backend.server.off("error", rejectListen);
      log(`Backend executando em http://localhost:${port}`);
      resolveListen();
    });
  });

  // A porta serial virtual é criada pelo sistema operacional após o pareamento
  // Bluetooth SPP. Sem a variável, o backend continua operando sem hardware.
  try {
  const bluetoothPath = process.env.MM_BLUETOOTH_SERIAL_PORT?.trim();
  if (process.env.MM_BLUETOOTH_TRANSPORT === "macos") {
    if(process.platform!=="darwin")throw new Error("Transporte macos exige macOS; use SPP serial neste sistema");
    backend.bluetoothBridge=startBluetoothNativeBridge({
      address:process.env.MM_BLUETOOTH_ADDRESS ?? "",
      executable:process.env.MM_BLUETOOTH_NATIVE_HELPER,
      onLine:(line)=>ingestBluetoothMessage(backend.gateway,line),
      onStatus:(status)=>ingestBluetoothMessage(backend.gateway,JSON.stringify({type:"bt_link_status",status})),
      log,
    });
    log("Ponte Bluetooth nativa do macOS habilitada.");
  } else if (bluetoothPath) {
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

  } catch (error) {
    await stopServer(backend);
    throw error;
  }
  backend.server.once("close",()=>{void backend.bluetoothBridge?.close();});
  return backend;
}


export async function stopServer(backend: BackendServer): Promise<void> {
  backend.robotControl?.close();
  await backend.bluetoothBridge?.close();
  for(const socket of backend.gateway.clients)socket.terminate();
  await new Promise<void>((resolve)=>backend.gateway.close(()=>resolve()));
  if(backend.server.listening)
    await new Promise<void>((resolve,reject)=>backend.server.close((error)=>error?reject(error):resolve()));
  await backend.closeDatabase?.();
}
