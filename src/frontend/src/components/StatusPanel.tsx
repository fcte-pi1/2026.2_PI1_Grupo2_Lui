import type { EstadoRobo } from "../types/telemetry";
import type { EstadoDashboard } from "../services/dashboardStore";
import { PidChart } from "./PidChart";
import { TENSAO_CRITICA, TENSAO_MAXIMA, TENSAO_MINIMA, bateriaCritica } from "../services/bateria";

// HU-09: indicadores de status e telemetria do robô

export function BatteryWidget({ tensao }: { tensao: number | null }) {
  const critica = bateriaCritica(tensao);
  const pct =
    tensao === null
      ? 0
      : Math.round(Math.min(1, Math.max(0, (tensao - TENSAO_MINIMA) / (TENSAO_MAXIMA - TENSAO_MINIMA))) * 100);
  return (
    <div
      className={`rounded-xl border p-4 ${
        critica ? "border-red-500 bg-red-500/10 animate-pulse" : "border-slate-300 dark:border-slate-700"
      }`}
    >
      <p className="text-sm">Bateria</p>
      <p className={`text-3xl font-semibold ${critica ? "text-red-600" : ""}`}>
        {tensao === null ? "--" : `${tensao.toFixed(2)} V`}
      </p>
      <div className="mt-2 h-2 rounded bg-slate-200 dark:bg-slate-700 overflow-hidden">
        <div
          className={`h-full ${critica ? "bg-red-600" : pct < 40 ? "bg-amber-500" : "bg-emerald-500"}`}
          style={{ width: `${pct}%` }}
        />
      </div>
      <p className="mt-1 text-xs">{tensao === null ? "sem leitura" : `${pct}% · limite seguro ${TENSAO_CRITICA} V`}</p>
    </div>
  );
}

export function BatteryAlert({ tensao }: { tensao: number | null }) {
  if (!bateriaCritica(tensao)) return null;
  return (
    <div role="alert" className="rounded-xl bg-red-600 text-white px-4 py-3 font-medium">
      ⚠ Bateria abaixo do limite seguro ({tensao!.toFixed(2)} V). Interrompa a corrida e recarregue para não danificar as células.
    </div>
  );
}

export function WheelSpeedCards({ esq, dir }: { esq: number | null; dir: number | null }) {
  const fmt = (v: number | null) => (v === null ? "--" : `${v.toFixed(2)} m/s`);
  return (
    <div className="grid grid-cols-2 gap-3">
      {[
        ["Roda esquerda", esq],
        ["Roda direita", dir],
      ].map(([rotulo, v]) => (
        <div key={rotulo as string} className="rounded-xl border border-slate-300 dark:border-slate-700 p-4">
          <p className="text-sm">{rotulo}</p>
          <p className="text-2xl font-semibold">{fmt(v as number | null)}</p>
        </div>
      ))}
    </div>
  );
}

const ESTADOS: Record<EstadoRobo, { rotulo: string; cor: string }> = {
  MAPPING: { rotulo: "Mapeando", cor: "bg-emerald-600" },
  NAVIGATING: { rotulo: "Navegando", cor: "bg-sky-600" },
  RETURNING: { rotulo: "Retornando", cor: "bg-amber-500" },
  FINISHED: { rotulo: "Concluído", cor: "bg-indigo-600" },
  STOPPED: { rotulo: "Parado", cor: "bg-slate-500" },
  ERROR: { rotulo: "Parado por erro", cor: "bg-red-600 animate-pulse" },
};

// HU-09 CA-03: estado lógico em destaque no topo
export function RobotStateBadge({ estado, motivo }: { estado: EstadoRobo | null; motivo: string | null }) {
  if (!estado) {
    return <span className="rounded-full bg-slate-400 px-4 py-1.5 text-white font-semibold">Sem conexão</span>;
  }
  const { rotulo, cor } = ESTADOS[estado] ?? { rotulo: estado, cor: "bg-slate-500" };
  return (
    <span className={`rounded-full px-4 py-1.5 text-white font-semibold ${cor}`}>
      {rotulo}
      {estado === "ERROR" && motivo ? ` · ${motivo}` : ""}
    </span>
  );
}

export function StatusPanel({ estado }: { estado: EstadoDashboard }) {
  return (
    <div className="flex flex-col gap-4">
      <BatteryWidget tensao={estado.tensaoBateria} />
      <WheelSpeedCards esq={estado.velocidadeEsq} dir={estado.velocidadeDir} />
      <div className="rounded-xl border border-slate-300 dark:border-slate-700 p-4">
        <p className="text-sm mb-2">
          Erro do controle PID{" "}
          <span className="font-semibold">
            {estado.historicoPid.length ? estado.historicoPid[estado.historicoPid.length - 1].erro.toFixed(3) : "--"}
          </span>
        </p>
        <PidChart pontos={estado.historicoPid} />
      </div>
    </div>
  );
}
