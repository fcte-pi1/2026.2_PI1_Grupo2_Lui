import type {
  Celula,
  Direcao,
  EstadoRobo,
  Paredes,
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
  // Trilha derivada das mensagens position; não substitui o trajeto persistido da HU-11.
  trajeto: Array<{ x: number; y: number }>;

  tensaoBateria: number | null;
  correnteBateriaA: number | null;
  leiturasSensorMm: { frente: number | null; esquerda: number | null; direita: number | null } | null;
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
    trajeto: [],
    tensaoBateria: null,
    correnteBateriaA: null,
    leiturasSensorMm: null,
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
      const atual = celulas[msg.y][msg.x];
      const anterior: Paredes = atual.walls ?? { north: null, south: null, east: null, west: null };
      // null significa "não medido", não "caminho livre". Uma mensagem parcial
      // não apaga conhecimento válido recebido em uma observação anterior.
      const walls: Paredes = {
        north: msg.walls.north ?? anterior.north,
        south: msg.walls.south ?? anterior.south,
        east: msg.walls.east ?? anterior.east,
        west: msg.walls.west ?? anterior.west,
      };
      celulas[msg.y][msg.x] = {
        x: msg.x,
        y: msg.y,
        visited: msg.visited || atual.visited,
        walls,
      };

      // Uma parede/passagem é compartilhada por duas células. Espelha apenas
      // lados medidos, mantendo desconhecidos intactos nas células vizinhas.
      const espelhar = (lado: keyof Paredes, nx: number, ny: number, oposto: keyof Paredes) => {
        const valor = walls[lado];
        if (valor === null || !dentroDaGrade(e, nx, ny)) return;
        const vizinha = celulas[ny][nx];
        const paredesVizinhas: Paredes = vizinha.walls ?? { north: null, south: null, east: null, west: null };
        celulas[ny][nx] = { ...vizinha, walls: { ...paredesVizinhas, [oposto]: valor } };
      };
      espelhar("north", msg.x, msg.y + 1, "south");
      espelhar("south", msg.x, msg.y - 1, "north");
      espelhar("east", msg.x + 1, msg.y, "west");
      espelhar("west", msg.x - 1, msg.y, "east");
      return { ...e, celulas };
    }

    case "position": {
      if (!dentroDaGrade(e, msg.x, msg.y)) return e;
      // a célula em que o robô está passa a contar como visitada
      const celulas = e.celulas.map((linha) => linha.slice());
      celulas[msg.y][msg.x] = { ...celulas[msg.y][msg.x], visited: true };
      const ultimo = e.trajeto[e.trajeto.length - 1];
      const mudouDeCelula = !ultimo || ultimo.x !== msg.x || ultimo.y !== msg.y;
      const trajeto = mudouDeCelula
        ? [...e.trajeto, { x: msg.x, y: msg.y }].slice(-5000)
        : e.trajeto;
      return {
        ...e,
        celulas,
        trajeto,
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
        correnteBateriaA: msg.battery_current_a ?? null,
        leiturasSensorMm: msg.sensor_distances_mm
          ? { frente: msg.sensor_distances_mm.front, esquerda: msg.sensor_distances_mm.left, direita: msg.sensor_distances_mm.right }
          : e.leiturasSensorMm,
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
