import type { Direcao } from "../types/telemetry";
import type { EstadoDashboard } from "../services/dashboardStore";

// HU-08: mapa 2D do labirinto em SVG.
// Convenção: (0,0) é a largada no canto inferior esquerdo e o Norte fica para cima.

const TAM = 60; // tamanho de cada célula em px (o SVG escala com a tela)
const MARGEM = 4;

const ROTACAO: Record<Direcao, number> = { N: 0, L: 90, S: 180, O: 270 };

interface Props {
  estado: EstadoDashboard;
}

export function MazeMap({ estado }: Props) {
  const { largura, altura, celulas, robo } = estado;
  const w = largura * TAM + MARGEM * 2;
  const h = altura * TAM + MARGEM * 2;
  // y do labirinto cresce para o Norte; no SVG cresce para baixo
  const topo = (y: number) => MARGEM + (altura - 1 - y) * TAM;
  const esq = (x: number) => MARGEM + x * TAM;

  return (
    <svg
      viewBox={`0 0 ${w} ${h}`}
      className="w-full h-auto max-h-[70vh]"
      role="img"
      aria-label={`Mapa do labirinto ${largura} por ${altura}`}
    >
      {/* células: inexplorada x visitada (CA-04) */}
      {celulas.flat().map((c) => (
        <rect
          key={`f-${c.x}-${c.y}`}
          x={esq(c.x)}
          y={topo(c.y)}
          width={TAM}
          height={TAM}
          className={c.visited ? "fill-sky-200 dark:fill-sky-900" : "fill-slate-200 dark:fill-slate-800"}
          opacity={c.visited ? 1 : 0.55}
        />
      ))}

      {/* grade de fundo (pontilhada) */}
      {celulas.flat().map((c) => (
        <rect
          key={`g-${c.x}-${c.y}`}
          x={esq(c.x)}
          y={topo(c.y)}
          width={TAM}
          height={TAM}
          fill="none"
          className="stroke-slate-400/40"
          strokeDasharray="3 4"
        />
      ))}

      {/* paredes descobertas (CA-02) */}
      {celulas.flat().map((c) => {
        if (!c.walls) return null;
        const x0 = esq(c.x);
        const y0 = topo(c.y);
        const linhas: [number, number, number, number][] = [];
        if (c.walls.north) linhas.push([x0, y0, x0 + TAM, y0]);
        if (c.walls.south) linhas.push([x0, y0 + TAM, x0 + TAM, y0 + TAM]);
        if (c.walls.west) linhas.push([x0, y0, x0, y0 + TAM]);
        if (c.walls.east) linhas.push([x0 + TAM, y0, x0 + TAM, y0 + TAM]);
        return linhas.map(([x1, y1, x2, y2], i) => (
          <line
            key={`p-${c.x}-${c.y}-${i}`}
            x1={x1}
            y1={y1}
            x2={x2}
            y2={y2}
            className="stroke-slate-900 dark:stroke-slate-100"
            strokeWidth={5}
            strokeLinecap="round"
          />
        ));
      })}

      {/* robô: destaque da célula atual + seta de orientação (CA-03) */}
      {robo && (
        <g>
          <rect
            x={esq(robo.x) + 3}
            y={topo(robo.y) + 3}
            width={TAM - 6}
            height={TAM - 6}
            rx={8}
            className="fill-amber-300/60 stroke-amber-500"
            strokeWidth={3}
          />
          <g
            transform={`translate(${esq(robo.x) + TAM / 2} ${topo(robo.y) + TAM / 2}) rotate(${ROTACAO[robo.heading]})`}
            style={{ transition: "transform 0.25s" }}
          >
            <polygon points="0,-18 13,14 0,7 -13,14" className="fill-rose-600" />
          </g>
        </g>
      )}
    </svg>
  );
}

export function MazeLegend() {
  const itens = [
    { cor: "bg-slate-200 opacity-60 dark:bg-slate-800", rotulo: "Inexplorada" },
    { cor: "bg-sky-200 dark:bg-sky-900", rotulo: "Visitada" },
    { cor: "bg-slate-900 dark:bg-slate-100 h-1.5!", rotulo: "Parede" },
    { cor: "bg-amber-300 border-2 border-amber-500", rotulo: "Robô" },
  ];
  return (
    <ul className="flex flex-wrap gap-x-5 gap-y-2 text-sm">
      {itens.map((i) => (
        <li key={i.rotulo} className="flex items-center gap-2">
          <span className={`inline-block w-5 h-5 rounded-sm ${i.cor}`} />
          {i.rotulo}
        </li>
      ))}
    </ul>
  );
}
