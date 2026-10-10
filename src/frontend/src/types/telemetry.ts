// Formatos das mensagens que o dashboard recebe pelo WebSocket.
// Os campos seguem o documento de Histórias de Usuário (HU-08 e HU-09).

export type Direcao = "N" | "S" | "L" | "O";

export interface Paredes {
  north: boolean;
  south: boolean;
  east: boolean;
  west: boolean;
}

export interface Celula {
  x: number;
  y: number;
  visited: boolean;
  // null = parede ainda não lida pelo robô
  walls: Paredes | null;
}

export type EstadoRobo =
  | "MAPPING"
  | "NAVIGATING"
  | "RETURNING"
  | "STOPPED"
  | "ERROR"
  | "FINISHED";

// HU-08: dimensões do labirinto da corrida
export interface MensagemLabirinto {
  type: "maze";
  width: number;
  height: number;
}

// HU-08: célula lida pelo robô (paredes descobertas)
export interface MensagemCelula {
  type: "cell";
  x: number;
  y: number;
  visited: boolean;
  walls: Paredes;
}

// HU-08: posição e orientação atuais do robô
export interface MensagemPosicao {
  type: "position";
  x: number;
  y: number;
  heading: Direcao;
}

// HU-09: indicadores de hardware e software
export interface MensagemTelemetria {
  type: "telemetry";
  battery_voltage: number;
  pid_error: number;
  wheels_speed: { left: number; right: number };
  robot_state: EstadoRobo;
  error_reason?: string;
  timestamp: number;
}

export type MensagemDashboard =
  | MensagemLabirinto
  | MensagemCelula
  | MensagemPosicao
  | MensagemTelemetria;
