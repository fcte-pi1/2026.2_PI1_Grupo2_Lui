import { StringDecoder } from "node:string_decoder";
import { SerialPort } from "serialport";

const MAX_FRAME_LENGTH = 64 * 1024;
const DEFAULT_RECONNECT_DELAY_MS = 3000;

export interface BluetoothSerialBridgeOptions {
  /** Porta serial virtual criada pelo sistema operacional para o perfil Bluetooth SPP. */
  path: string;
  /** Taxa exigida pela API SerialPort; em portas SPP virtuais o valor não altera o rádio. */
  baudRate?: number;
  reconnectDelayMs?: number;
  onLine: (line: string) => void;
  log?: (message: string) => void;
}

export interface BluetoothSerialBridge {
  close(): Promise<void>;
}

/**
 * Abre a porta serial virtual do Bluetooth Classic SPP e reconecta quando o
 * dispositivo ou a porta do sistema operacional é desconectada. Cada pacote
 * do firmware é um objeto JSON terminado por '\\n'.
 */
export function startBluetoothSerialBridge(
  options: BluetoothSerialBridgeOptions,
): BluetoothSerialBridge {
  const log = options.log ?? console.log;
  const port = new SerialPort({
    path: options.path,
    baudRate: options.baudRate ?? 115200,
    autoOpen: false,
  });

  let disposed = false;
  let opening = false;
  let reconnectTimer: NodeJS.Timeout | undefined;
  let decoder = new StringDecoder("utf8");
  let pendingLine = "";
  let discardingOversizedFrame = false;

  const resetFraming = () => {
    decoder = new StringDecoder("utf8");
    pendingLine = "";
    discardingOversizedFrame = false;
  };

  const scheduleReconnect = () => {
    if (disposed || reconnectTimer) return;
    reconnectTimer = setTimeout(() => {
      reconnectTimer = undefined;
      openPort();
    }, options.reconnectDelayMs ?? DEFAULT_RECONNECT_DELAY_MS);
    reconnectTimer.unref();
  };

  const openPort = () => {
    if (disposed || opening || port.isOpen) return;
    opening = true;
    port.open((error) => {
      opening = false;
      if (error) {
        log(`Bluetooth SPP: não foi possível abrir ${options.path}: ${error.message}`);
        scheduleReconnect();
        return;
      }
      if (disposed) {
        port.close(() => undefined);
        return;
      }
      log(`Bluetooth SPP: porta ${options.path} aberta; aguardando dados do ESP32.`);
    });
  };

  port.on("data", (chunk: Buffer) => {
    const decoded = decoder.write(chunk);
    for (const character of decoded) {
      if (character === "\n") {
        if (!discardingOversizedFrame) {
          const frame = pendingLine.trim();
          if (frame.length > 0) {
            try {
              options.onLine(frame);
            } catch (error) {
              log(`Bluetooth SPP: erro ao encaminhar pacote: ${String(error)}`);
            }
          }
        }
        pendingLine = "";
        discardingOversizedFrame = false;
        continue;
      }

      if (discardingOversizedFrame) continue;
      pendingLine += character;
      if (pendingLine.length > MAX_FRAME_LENGTH) {
        pendingLine = "";
        discardingOversizedFrame = true;
        log(`Bluetooth SPP: frame excedeu ${MAX_FRAME_LENGTH} caracteres e foi descartado.`);
      }
    }
  });

  port.on("error", (error) => {
    log(`Bluetooth SPP: erro na porta ${options.path}: ${error.message}`);
  });

  port.on("close", () => {
    resetFraming();
    if (!disposed) {
      log(`Bluetooth SPP: porta ${options.path} fechada; nova tentativa em ${options.reconnectDelayMs ?? DEFAULT_RECONNECT_DELAY_MS} ms.`);
      scheduleReconnect();
    }
  });

  openPort();

  return {
    close: () => {
      disposed = true;
      if (reconnectTimer) clearTimeout(reconnectTimer);
      reconnectTimer = undefined;
      if (!port.isOpen) return Promise.resolve();
      return new Promise<void>((resolve) => {
        port.close(() => resolve());
      });
    },
  };
}
