import { useEffect, useState } from "react";
import { useDashboard } from "../hooks/useDashboard";
import { dashboardStore } from "../services/dashboardStore";
import { connectTelemetry } from "../services/websocket";
import { iniciarSimulador } from "../services/simulador";
import { MazeLegend, MazeMap } from "../components/MazeMap";
import { BatteryAlert, RobotStateBadge, StatusPanel } from "../components/StatusPanel";
import type { MensagemDashboard } from "../types/telemetry";

type Fonte = "ws" | "simulador";

const TAMANHOS = [
  { rotulo: "4x4", largura: 4, altura: 4 },
  { rotulo: "8x4", largura: 8, altura: 4 },
  { rotulo: "12x4", largura: 12, altura: 4 },
];

const TIPOS = ["maze", "cell", "position", "telemetry"];

function ehMensagemDashboard(data: unknown): data is MensagemDashboard {
  return typeof data === "object" && data !== null && TIPOS.includes((data as { type?: string }).type ?? "");
}

export function Dashboard() {
  const estado = useDashboard();
  // ?fonte=simulador na URL já abre em modo simulação
  const [fonte, setFonte] = useState<Fonte>(() =>
    new URLSearchParams(window.location.search).get("fonte") === "simulador" ? "simulador" : "ws",
  );
  const [tamanho, setTamanho] = useState(1);
  const [rodada, setRodada] = useState(0);

  useEffect(() => {
    const { largura, altura } = TAMANHOS[tamanho];
    dashboardStore.reset(largura, altura);

    if (fonte === "simulador") {
      return iniciarSimulador({ largura, altura }, dashboardStore.dispatch);
    }
    const socket = connectTelemetry((data) => {
      if (ehMensagemDashboard(data)) dashboardStore.dispatch(data);
    });
    return () => socket.close();
  }, [fonte, tamanho, rodada]);

  return (
    <div className="min-h-screen bg-slate-50 text-slate-800 dark:bg-slate-950 dark:text-slate-200">
      <header className="sticky top-0 z-10 flex flex-wrap items-center gap-4 border-b border-slate-200 bg-white/90 px-6 py-3 backdrop-blur dark:border-slate-800 dark:bg-slate-900/90">
        <h1 className="text-xl font-bold text-slate-900 dark:text-white">MicroMouse · Painel da corrida</h1>
        <RobotStateBadge estado={estado.estadoRobo} motivo={estado.motivoErro} />
        <div className="ml-auto flex flex-wrap items-center gap-2 text-sm">
          <select
            value={fonte}
            onChange={(e) => setFonte(e.target.value as Fonte)}
            className="rounded-md border border-slate-300 bg-transparent px-2 py-1 dark:border-slate-700"
          >
            <option value="ws">Robô (WebSocket)</option>
            <option value="simulador">Simulação</option>
          </select>
          <select
            value={tamanho}
            onChange={(e) => setTamanho(Number(e.target.value))}
            className="rounded-md border border-slate-300 bg-transparent px-2 py-1 dark:border-slate-700"
          >
            {TAMANHOS.map((t, i) => (
              <option key={t.rotulo} value={i}>
                Labirinto {t.rotulo}
              </option>
            ))}
          </select>
          <button
            onClick={() => setRodada((r) => r + 1)}
            className="rounded-md bg-slate-900 px-3 py-1 text-white dark:bg-white dark:text-slate-900"
          >
            Reiniciar
          </button>
        </div>
      </header>

      <main className="mx-auto flex max-w-7xl flex-col gap-4 p-6">
        <BatteryAlert tensao={estado.tensaoBateria} />
        <div className="grid gap-6 lg:grid-cols-[3fr_2fr]">
          <section className="flex flex-col gap-3 rounded-2xl border border-slate-200 bg-white p-4 dark:border-slate-800 dark:bg-slate-900">
            <h2 className="font-semibold text-slate-900 dark:text-white">
              Mapa do labirinto ({estado.largura}x{estado.altura})
            </h2>
            <MazeMap estado={estado} />
            <MazeLegend />
            {estado.robo && (
              <p className="text-sm">
                Robô na célula ({estado.robo.x}, {estado.robo.y}) · virado para {estado.robo.heading}
              </p>
            )}
          </section>
          <section className="flex flex-col gap-3 rounded-2xl border border-slate-200 bg-white p-4 dark:border-slate-800 dark:bg-slate-900">
            <h2 className="font-semibold text-slate-900 dark:text-white">Status e telemetria</h2>
            <StatusPanel estado={estado} />
          </section>
        </div>
      </main>
    </div>
  );
}
