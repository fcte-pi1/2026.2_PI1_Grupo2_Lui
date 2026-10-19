import { useEffect, useState } from "react";
import { EventLog } from "./EventLog";
import { getSessionLogs, type LogEvento, type Sessao } from "../services/api";
import { formatarData, formatarDuracao, sessaoConcluida } from "../utils/formatters";

interface SessionOverviewProps {
  sessoes: Sessao[] | null;
  erroSessoes: string | null;
  carregandoSessoes: boolean;
  versaoSessoes: number;
  onAtualizarSessoes: () => void;
}

const mensagemErro = (erro: unknown) =>
  erro instanceof Error ? erro.message : "Erro ao consultar os eventos";

/** Resumo HU-11/HU-12 dentro do painel ao vivo HU-08/HU-09. */
export function SessionOverview({
  sessoes,
  erroSessoes,
  carregandoSessoes,
  versaoSessoes,
  onAtualizarSessoes,
}: SessionOverviewProps) {
  const ultimaSessao = sessoes?.[0] ?? null;
  const ultimaSessaoId = ultimaSessao?.id_sessao ?? null;
  const [eventos, setEventos] = useState<LogEvento[] | null>(null);
  const [erroEventos, setErroEventos] = useState<string | null>(null);
  const [carregandoEventos, setCarregandoEventos] = useState(false);

  useEffect(() => {
    if (ultimaSessaoId === null) {
      setEventos(null);
      setErroEventos(null);
      setCarregandoEventos(false);
      return;
    }

    let cancelado = false;
    setCarregandoEventos(true);
    setErroEventos(null);
    getSessionLogs(ultimaSessaoId, undefined, { limite: 5, recentes: true })
      .then((dados) => {
        if (!cancelado) setEventos(dados);
      })
      .catch((erro: unknown) => {
        if (!cancelado) setErroEventos(mensagemErro(erro));
      })
      .finally(() => {
        if (!cancelado) setCarregandoEventos(false);
      });

    return () => {
      cancelado = true;
    };
  }, [ultimaSessaoId, versaoSessoes]);

  const concluidas = sessoes?.filter(sessaoConcluida).length ?? 0;
  const resumo = sessoes?.slice(0, 4) ?? [];

  return (
    <section className="flex flex-col gap-4 rounded-2xl border border-slate-200 bg-white p-4 dark:border-slate-800 dark:bg-slate-900">
      <header className="flex flex-wrap items-center justify-between gap-3">
        <div>
          <h2 className="font-semibold text-slate-900 dark:text-white">Execuções e diagnósticos</h2>
          <p className="mt-1 text-sm text-slate-500 dark:text-slate-400">
            Histórico de sessões, trajetos e eventos recentes do robô.
          </p>
        </div>
        <button
          type="button"
          onClick={onAtualizarSessoes}
          disabled={carregandoSessoes}
          className="rounded-md border border-slate-300 px-3 py-2 text-sm font-medium hover:bg-slate-100 disabled:cursor-wait disabled:opacity-50 dark:border-slate-700 dark:hover:bg-slate-800"
        >
          {carregandoSessoes ? "Atualizando…" : "Atualizar histórico"}
        </button>
      </header>

      {erroSessoes && (
        <p role="alert" className="rounded-lg border border-amber-500/30 bg-amber-500/10 px-3 py-2 text-sm text-amber-700 dark:text-amber-300">
          Não foi possível carregar as sessões: {erroSessoes}
        </p>
      )}

      <div className="grid gap-4 lg:grid-cols-[minmax(0,3fr)_minmax(0,2fr)]">
        <div className="min-w-0">
          <div className="mb-3 grid grid-cols-2 gap-3">
            <div className="rounded-xl border border-slate-200 p-3 dark:border-slate-800">
              <p className="text-xs uppercase tracking-wide text-slate-500 dark:text-slate-400">Sessões registradas</p>
              <p className="mt-1 text-2xl font-semibold text-slate-900 dark:text-white">
                {sessoes === null ? "—" : sessoes.length}
              </p>
            </div>
            <div className="rounded-xl border border-slate-200 p-3 dark:border-slate-800">
              <p className="text-xs uppercase tracking-wide text-slate-500 dark:text-slate-400">Desafios concluídos</p>
              <p className="mt-1 text-2xl font-semibold text-slate-900 dark:text-white">
                {sessoes === null ? "—" : concluidas}
              </p>
            </div>
          </div>

          <h3 className="mb-2 text-sm font-semibold text-slate-800 dark:text-slate-200">Sessões recentes</h3>
          {carregandoSessoes && sessoes === null && (
            <p className="py-4 text-sm text-slate-500 dark:text-slate-400">Carregando sessões…</p>
          )}
          {sessoes && resumo.length === 0 && (
            <p className="py-4 text-sm text-slate-500 dark:text-slate-400">Nenhuma sessão registrada ainda.</p>
          )}
          {resumo.length > 0 && (
            <ul className="divide-y divide-slate-200 dark:divide-slate-800">
              {resumo.map((sessao) => {
                const sucesso = sessaoConcluida(sessao);
                return (
                  <li key={sessao.id_sessao} className="flex flex-wrap items-center justify-between gap-3 py-3 first:pt-1">
                    <div className="min-w-0">
                      <p className="text-sm font-medium text-slate-900 dark:text-slate-100">
                        #{sessao.id_sessao} · {formatarData(sessao.data_inicio)}
                      </p>
                      <p className="mt-1 text-xs text-slate-500 dark:text-slate-400">
                        {sessao.tipo_labirinto ?? "Labirinto não informado"}
                        {sessao.numero_tentativa !== null ? ` · tentativa ${sessao.numero_tentativa}` : ""}
                        {sessao.consumo_bateria_mah !== null ? ` · ${sessao.consumo_bateria_mah.toFixed(1)} mAh` : ""}
                      </p>
                    </div>
                    <div className="text-right">
                      <span className={`rounded-full px-2 py-1 text-xs font-semibold ${sucesso ? "bg-emerald-500/15 text-emerald-700 dark:text-emerald-300" : "bg-slate-500/15 text-slate-600 dark:text-slate-300"}`}>
                        {sessao.status_conclusao}
                      </span>
                      <p className="mt-1 text-xs text-slate-500 dark:text-slate-400">
                        {formatarDuracao(sessao.tempo_total_segundos)}
                      </p>
                    </div>
                  </li>
                );
              })}
            </ul>
          )}
        </div>

        <div className="min-w-0 rounded-xl border border-slate-200 dark:border-slate-800">
          <div className="border-b border-slate-200 p-3 dark:border-slate-800">
            <h3 className="text-sm font-semibold text-slate-900 dark:text-slate-100">Últimos eventos</h3>
            <p className="mt-1 text-xs text-slate-500 dark:text-slate-400">
              {ultimaSessao ? `Sessão #${ultimaSessao.id_sessao} · até 5 registros mais recentes` : "Aguardando uma sessão"}
            </p>
          </div>
          <EventLog
            logs={eventos ?? []}
            carregando={carregandoEventos}
            erro={erroEventos}
          />
        </div>
      </div>
    </section>
  );
}
