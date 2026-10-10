import type {
  Direcao,
  EstadoRobo,
  MensagemDashboard,
  Paredes,
} from "../types/telemetry";

// Simulador de corrida para testar o dashboard sem o robô.
// Gera um labirinto aleatório, faz o robô explorar em profundidade até o
// objetivo (canto oposto) e voltar para a largada, emitindo as mesmas
// mensagens que o backend vai repassar pelo WebSocket.

const DELTA: Record<Direcao, [number, number]> = {
  N: [0, 1],
  S: [0, -1],
  L: [1, 0],
  O: [-1, 0],
};
const OPOSTA: Record<Direcao, Direcao> = { N: "S", S: "N", L: "O", O: "L" };
const CHAVE: Record<Direcao, keyof Paredes> = {
  N: "north",
  S: "south",
  L: "east",
  O: "west",
};

function gerarLabirinto(largura: number, altura: number): Paredes[][] {
  const paredes: Paredes[][] = Array.from({ length: altura }, () =>
    Array.from({ length: largura }, () => ({
      north: true,
      south: true,
      east: true,
      west: true,
    })),
  );
  const visitado = new Set<string>(["0,0"]);
  const pilha: [number, number][] = [[0, 0]];
  while (pilha.length) {
    const [x, y] = pilha[pilha.length - 1];
    const opcoes = (Object.keys(DELTA) as Direcao[]).filter((d) => {
      const nx = x + DELTA[d][0];
      const ny = y + DELTA[d][1];
      return (
        nx >= 0 && ny >= 0 && nx < largura && ny < altura &&
        !visitado.has(`${nx},${ny}`)
      );
    });
    if (!opcoes.length) {
      pilha.pop();
      continue;
    }
    const d = opcoes[Math.floor(Math.random() * opcoes.length)];
    const nx = x + DELTA[d][0];
    const ny = y + DELTA[d][1];
    paredes[y][x][CHAVE[d]] = false;
    paredes[ny][nx][CHAVE[OPOSTA[d]]] = false;
    visitado.add(`${nx},${ny}`);
    pilha.push([nx, ny]);
  }
  return paredes;
}

// Caminho de exploração (lista de passos) até o objetivo e de volta
function planejarCorrida(paredes: Paredes[][], largura: number, altura: number) {
  const objetivo = `${largura - 1},${altura - 1}`;
  const passos: { x: number; y: number; heading: Direcao; estado: EstadoRobo }[] = [];
  const visitado = new Set<string>(["0,0"]);
  const caminho: { x: number; y: number; veioPor: Direcao | null }[] = [
    { x: 0, y: 0, veioPor: null },
  ];

  while (caminho.length) {
    const atual = caminho[caminho.length - 1];
    if (`${atual.x},${atual.y}` === objetivo) break;
    const d = (Object.keys(DELTA) as Direcao[]).find((dir) => {
      const nx = atual.x + DELTA[dir][0];
      const ny = atual.y + DELTA[dir][1];
      return !paredes[atual.y][atual.x][CHAVE[dir]] && !visitado.has(`${nx},${ny}`);
    });
    if (d) {
      const nx = atual.x + DELTA[d][0];
      const ny = atual.y + DELTA[d][1];
      visitado.add(`${nx},${ny}`);
      caminho.push({ x: nx, y: ny, veioPor: d });
      passos.push({ x: nx, y: ny, heading: d, estado: "MAPPING" });
    } else {
      // beco sem saída: volta uma célula
      caminho.pop();
      const anterior = caminho[caminho.length - 1];
      if (anterior && atual.veioPor) {
        passos.push({
          x: anterior.x,
          y: anterior.y,
          heading: OPOSTA[atual.veioPor],
          estado: "MAPPING",
        });
      }
    }
  }

  // retorno para a largada pelo caminho encontrado
  for (let i = caminho.length - 1; i > 0; i--) {
    const anterior = caminho[i - 1];
    passos.push({
      x: anterior.x,
      y: anterior.y,
      heading: OPOSTA[caminho[i].veioPor!],
      estado: "RETURNING",
    });
  }
  return passos;
}

export interface OpcoesSimulador {
  largura: number;
  altura: number;
  msPorCelula?: number;
}

export function iniciarSimulador(
  { largura, altura, msPorCelula = 600 }: OpcoesSimulador,
  emitir: (msg: MensagemDashboard) => void,
) {
  const paredes = gerarLabirinto(largura, altura);
  const passos = planejarCorrida(paredes, largura, altura);
  const inicio = Date.now();
  let bateria = 12.4;
  let estado: EstadoRobo = "MAPPING";
  let indice = 0;

  emitir({ type: "maze", width: largura, height: altura });
  emitir({ type: "cell", x: 0, y: 0, visited: true, walls: paredes[0][0] });
  emitir({ type: "position", x: 0, y: 0, heading: "N" });

  const timerMovimento = setInterval(() => {
    if (indice >= passos.length) {
      estado = "FINISHED";
      clearInterval(timerMovimento);
      return;
    }
    const p = passos[indice++];
    estado = p.estado;
    emitir({ type: "cell", x: p.x, y: p.y, visited: true, walls: paredes[p.y][p.x] });
    emitir({ type: "position", x: p.x, y: p.y, heading: p.heading });
  }, msPorCelula);

  const timerTelemetria = setInterval(() => {
    const t = (Date.now() - inicio) / 1000;
    // descarga acelerada para dar para ver o alerta de bateria na simulação
    bateria = Math.max(9.8, bateria - 0.012);
    const parado = estado === "FINISHED";
    const base = parado ? 0 : 0.45;
    const erro = parado ? 0 : 0.08 * Math.sin(t * 3) + (Math.random() - 0.5) * 0.03;
    emitir({
      type: "telemetry",
      battery_voltage: Number(bateria.toFixed(2)),
      pid_error: Number(erro.toFixed(3)),
      wheels_speed: {
        left: Number((base + erro).toFixed(2)),
        right: Number((base - erro).toFixed(2)),
      },
      robot_state: estado,
      timestamp: Date.now(),
    });
  }, 200);

  return () => {
    clearInterval(timerMovimento);
    clearInterval(timerTelemetria);
  };
}
