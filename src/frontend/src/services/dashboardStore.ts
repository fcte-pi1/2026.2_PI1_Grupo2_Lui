import type {
  Celula,
  Direcao,
  EstadoRobo,
  MensagemDashboard,
} from "../types/telemetry";

// Estado global do dashboard. Fica fora dos componentes para não se perder
// quando o operador troca de aba (HU-08, "preservação de estado").

export interface PontoPid {
  timestamp: number;
  erro: number;
}

export interface EstadoDashboard {
  largura: number;
  altura: number;
  celulas: Celula[][]; // celulas[y][x]
  robo: { x: number; y: number; heading: Direcao } | null;

  tensaoBateria: number | null;
  velocidadeEsq: number | null;
  velocidadeDir: number | null;
  estadoRobo: EstadoRobo | null;
  motivoErro: string | null;
  historicoPid: PontoPid[];
  ultimaAtualizacao: number | null;
}

// Quantos pontos do erro PID o gráfico guarda (janela deslizante)
export const MAX_PONTOS_PID = 120;

export function criarGrade(largura: number, altura: number): Celula[][] {
  return Array.from({ length: altura }, (_, y) =>
    Array.from({ length: largura }, (_, x) => ({
      x,
      y,
      visited: false,
      walls: null,
    })),
  );
}

export function estadoInicial(largura = 4, altura = 4): EstadoDashboard {
  return {
    largura,
    altura,
    celulas: criarGrade(largura, altura),
    robo: null,
    tensaoBateria: null,
    velocidadeEsq: null,
    velocidadeDir: null,
    estadoRobo: null,
    motivoErro: null,
    historicoPid: [],
    ultimaAtualizacao: null,
  };
}

function dentroDaGrade(e: EstadoDashboard, x: number, y: number) {
  return x >= 0 && y >= 0 && x < e.largura && y < e.altura;
}

// Função pura: recebe o estado atual e uma mensagem, devolve o novo estado.
export function aplicarMensagem(
  e: EstadoDashboard,
  msg: MensagemDashboard,
): EstadoDashboard {
  switch (msg.type) {
    case "maze":
      return estadoInicial(msg.width, msg.height);

    case "cell": {
      if (!dentroDaGrade(e, msg.x, msg.y)) return e;
      const celulas = e.celulas.map((linha) => linha.slice());
      celulas[msg.y][msg.x] = {
        x: msg.x,
        y: msg.y,
        visited: msg.visited,
        walls: msg.walls,
      };
      return { ...e, celulas };
    }

    case "position": {
      if (!dentroDaGrade(e, msg.x, msg.y)) return e;
      // a célula em que o robô está passa a contar como visitada
      const celulas = e.celulas.map((linha) => linha.slice());
      celulas[msg.y][msg.x] = { ...celulas[msg.y][msg.x], visited: true };
      return {
        ...e,
        celulas,
        robo: { x: msg.x, y: msg.y, heading: msg.heading },
      };
    }

    case "telemetry": {
      const historicoPid = [
        ...e.historicoPid,
        { timestamp: msg.timestamp, erro: msg.pid_error },
      ].slice(-MAX_PONTOS_PID);
      return {
        ...e,
        tensaoBateria: msg.battery_voltage,
        velocidadeEsq: msg.wheels_speed.left,
        velocidadeDir: msg.wheels_speed.right,
        estadoRobo: msg.robot_state,
        motivoErro: msg.error_reason ?? null,
        historicoPid,
        ultimaAtualizacao: msg.timestamp,
      };
    }

    default:
      return e;
  }
}

// --- store simples (sem biblioteca externa) ---

let estado = estadoInicial();
const ouvintes = new Set<() => void>();

export const dashboardStore = {
  getState: () => estado,
  subscribe(ouvinte: () => void) {
    ouvintes.add(ouvinte);
    return () => ouvintes.delete(ouvinte);
  },
  dispatch(msg: MensagemDashboard) {
    estado = aplicarMensagem(estado, msg);
    ouvintes.forEach((o) => o());
  },
  reset(largura = estado.largura, altura = estado.altura) {
    estado = estadoInicial(largura, altura);
    ouvintes.forEach((o) => o());
  },
};
