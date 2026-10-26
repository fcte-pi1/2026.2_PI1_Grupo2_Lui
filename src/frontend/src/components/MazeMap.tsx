import type { Direcao } from "../types/telemetry";
import type { EstadoDashboard } from "../services/dashboardStore";
import { useEffect, useState } from "react";

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
  const [zoom, setZoom] = useState(1);
  const [seguirRobo, setSeguirRobo] = useState(false);
  const [centro, setCentro] = useState<{ x: number; y: number } | null>(null);
  const viewW = w / zoom;
  const viewH = h / zoom;

  const centroX = centro?.x ?? w / 2;
  const centroY = centro?.y ?? h / 2;

  const viewX = Math.max(0, Math.min(w - viewW, centroX - viewW / 2));
  const viewY = Math.max(0, Math.min(h - viewH, centroY - viewH / 2));

  const aproximar = () => setZoom((atual) => Math.min(4, atual * 1.25));
  const afastar = () => setZoom((atual) => Math.max(1, atual / 1.25));

  const centralizarRobo = () => {
    if (!robo) return;

    setCentro({
      x: esq(robo.x) + TAM / 2,
      y: topo(robo.y) + TAM / 2,
    });
  };

  useEffect(() => {
    if (seguirRobo && robo) {
      setCentro({
        x: esq(robo.x) + TAM / 2,
        y: topo(robo.y) + TAM / 2,
      });
    }
  }, [seguirRobo, robo?.x, robo?.y]);

  return (
    <div className="space-y-3">
      <div className="flex flex-wrap items-center justify-between gap-3 rounded-xl border border-slate-200 bg-white p-3 shadow-sm dark:border-slate-700 dark:bg-slate-900">
        <div className="flex items-center gap-2">
          <span className="text-sm font-semibold text-slate-600 dark:text-slate-300">
            ZOOM
          </span>

          <button
            type="button"
            onClick={afastar}
            disabled={zoom <= 1}
            aria-label="Afastar mapa"
            title="Afastar mapa"
            className="flex h-9 w-9 items-center justify-center rounded-lg border border-slate-200 bg-slate-50 text-lg font-semibold text-slate-700 transition hover:bg-slate-100 disabled:cursor-not-allowed disabled:opacity-40 dark:border-slate-700 dark:bg-slate-800 dark:text-slate-100 dark:hover:bg-slate-700"
          >
            −
          </button>

          <span className="min-w-[58px] text-center text-sm font-semibold tabular-nums text-slate-800 dark:text-slate-100">
            {Math.round(zoom * 100)}%
          </span>

          <button
            type="button"
            onClick={aproximar}
            disabled={zoom >= 4}
            aria-label="Aproximar mapa"
            title="Aproximar mapa"
            className="flex h-9 w-9 items-center justify-center rounded-lg border border-slate-200 bg-slate-50 text-lg font-semibold text-slate-700 transition hover:bg-slate-100 disabled:cursor-not-allowed disabled:opacity-40 dark:border-slate-700 dark:bg-slate-800 dark:text-slate-100 dark:hover:bg-slate-700"
          >
            +
          </button>
        </div>

        <div className="flex flex-wrap items-center gap-2">
          <button
            type="button"
            onClick={centralizarRobo}
            disabled={!robo}
            className="rounded-lg border border-slate-200 px-3 py-2 text-sm font-medium text-slate-700 transition hover:bg-slate-100 disabled:cursor-not-allowed disabled:opacity-40 dark:border-slate-700 dark:text-slate-200 dark:hover:bg-slate-800"
          >
            Centralizar
          </button>

          <button
            type="button"
            onClick={() => {
              if (!seguirRobo) centralizarRobo();
              setSeguirRobo((atual) => !atual);
            }}
            disabled={!robo}
            aria-pressed={seguirRobo}
            className={`rounded-lg px-3 py-2 text-sm font-semibold transition disabled:cursor-not-allowed disabled:opacity-40 ${
              seguirRobo
              ? "border border-slate-400 bg-slate-200 text-slate-900 hover:bg-slate-300 dark:border-slate-500 dark:bg-slate-700 dark:text-white dark:hover:bg-slate-600"
              : "border border-slate-200 text-slate-700 hover:bg-slate-100 dark:border-slate-700 dark:text-slate-200 dark:hover:bg-slate-800"
              }`}
          >
            {seguirRobo ? "Seguindo robô" : "Seguir robô"}
          </button>
        </div>
      </div>

      <svg
        viewBox={`${viewX} ${viewY} ${viewW} ${viewH}`}
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

        {/* Trilha ao vivo derivada das mensagens de posição (HU-08); paredes ficam sobre a trilha. */}
        {estado.trajeto.length > 1 && (
          <polyline
            points={estado.trajeto
              .map((ponto) => `${esq(ponto.x) + TAM / 2},${topo(ponto.y) + TAM / 2}`)
              .join(" ")}
            fill="none"
            className="stroke-violet-600 dark:stroke-violet-400"
            strokeWidth={7}
            strokeLinecap="round"
            strokeLinejoin="round"
            opacity={0.78}
          />
        )}

        {/* paredes descobertas (CA-02) */}
        {celulas.flat().map((c) => {
          if (!c.walls) return null;
          const x0 = esq(c.x);
          const y0 = topo(c.y);
          const linhas: [number, number, number, number][] = [];
          const desconhecidas: [number, number, number, number][] = [];
          if (c.walls.north === true) linhas.push([x0, y0, x0 + TAM, y0]);
          else if (c.walls.north === null) desconhecidas.push([x0, y0, x0 + TAM, y0]);
          if (c.walls.south === true) linhas.push([x0, y0 + TAM, x0 + TAM, y0 + TAM]);
          else if (c.walls.south === null) desconhecidas.push([x0, y0 + TAM, x0 + TAM, y0 + TAM]);
          if (c.walls.west === true) linhas.push([x0, y0, x0, y0 + TAM]);
          else if (c.walls.west === null) desconhecidas.push([x0, y0, x0, y0 + TAM]);
          if (c.walls.east === true) linhas.push([x0 + TAM, y0, x0 + TAM, y0 + TAM]);
          else if (c.walls.east === null) desconhecidas.push([x0 + TAM, y0, x0 + TAM, y0 + TAM]);
          return (
            <>
              {desconhecidas.map(([x1, y1, x2, y2], i) => (
                <line key={`u-${c.x}-${c.y}-${i}`} x1={x1} y1={y1} x2={x2} y2={y2}
                  className="stroke-slate-500/70 dark:stroke-slate-400/70" strokeWidth={2} strokeDasharray="4 4" />
              ))}
              {linhas.map(([x1, y1, x2, y2], i) => (
                <line key={`p-${c.x}-${c.y}-${i}`} x1={x1} y1={y1} x2={x2} y2={y2}
                  className="stroke-slate-900 dark:stroke-slate-100" strokeWidth={5} strokeLinecap="round" />
              ))}
            </>
          );
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
    </div>
  );
}

export function MazeLegend() {
  const itens = [
    { cor: "bg-slate-200 opacity-60 dark:bg-slate-800", rotulo: "Inexplorada" },
    { cor: "bg-sky-200 dark:bg-sky-900", rotulo: "Visitada" },
    { cor: "bg-violet-600 dark:bg-violet-400 h-1.5!", rotulo: "Trajeto" },
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
