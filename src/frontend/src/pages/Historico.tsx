import { useEffect, useState } from 'react';
import { EventLog } from '../components/EventLog';
import {
  getSessionLogs,
  getSessionNavigation,
  NIVEIS,
  type LogEvento,
  type Nivel,
  type PontoPercurso,
  type Sessao,
} from '../services/api';
import { formatarData, formatarDuracao, formatarTempo } from '../utils/formatters';

type Filtro = Nivel | 'TODOS';

interface Resultado<T> {
  chave: string;
  dados: T;
  erro: string | null;
}

interface HistoricoProps {
  ativo: boolean;
  sessoes: Sessao[] | null;
  erroSessoes: string | null;
  carregandoSessoes: boolean;
  versaoSessoes: number;
  onAtualizarSessoes: () => void;
  onVoltar: () => void;
}

const mensagemErro = (e: unknown) =>
  e instanceof Error ? e.message : 'Erro ao comunicar com o servidor';

export default function Historico({
  ativo,
  sessoes,
  erroSessoes,
  carregandoSessoes,
  versaoSessoes,
  onAtualizarSessoes,
  onVoltar,
}: HistoricoProps) {
  const [idSelecionada, setIdSelecionada] = useState<number | null>(null);
  const [filtro, setFiltro] = useState<Filtro>('TODOS');
  const [logs, setLogs] = useState<Resultado<LogEvento[]> | null>(null);
  const [pontos, setPontos] = useState<Resultado<PontoPercurso[]> | null>(null);

  // Seleciona a sessão mais recente ao abrir a tela, sem sobrescrever a escolha do usuário.
  useEffect(() => {
    if (idSelecionada === null && sessoes && sessoes.length > 0) {
      setIdSelecionada(sessoes[0].id_sessao);
    }
  }, [idSelecionada, sessoes]);

  const chaveLogs = `${idSelecionada}:${filtro}`;
  const chavePontos = idSelecionada === null ? '' : String(idSelecionada);
  const lista = sessoes ?? [];
  const selecionada = lista.find((s) => s.id_sessao === idSelecionada);

  useEffect(() => {
    if (!ativo || idSelecionada === null) return;
    let cancelado = false;
    const nivel = filtro === 'TODOS' ? undefined : filtro;
    getSessionLogs(idSelecionada, nivel)
      .then((dados) => !cancelado && setLogs({ chave: chaveLogs, dados, erro: null }))
      .catch((e) => !cancelado && setLogs({ chave: chaveLogs, dados: [], erro: mensagemErro(e) }));
    return () => {
      cancelado = true;
    };
  }, [ativo, idSelecionada, filtro, chaveLogs, versaoSessoes]);

  useEffect(() => {
    if (!ativo || idSelecionada === null) return;
    let cancelado = false;
    getSessionNavigation(idSelecionada)
      .then((dados) => !cancelado && setPontos({ chave: chavePontos, dados, erro: null }))
      .catch((e) => !cancelado && setPontos({ chave: chavePontos, dados: [], erro: mensagemErro(e) }));
    return () => {
      cancelado = true;
    };
  }, [ativo, idSelecionada, chavePontos, versaoSessoes]);

  const carregandoLogs = idSelecionada !== null && logs?.chave !== chaveLogs;

  return (
    <main className="min-h-screen bg-slate-950 text-slate-200">
      <header className="sticky top-0 z-10 flex flex-wrap items-center justify-between gap-4 border-b border-slate-800 bg-slate-900/95 px-6 py-4 backdrop-blur">
        <div>
          <h1 className="text-xl font-bold text-white">Histórico de sessões e diagnósticos</h1>
          <p className="mt-1 text-sm text-slate-400">HU-11 · trajetos persistidos e HU-12 · eventos cronológicos</p>
        </div>
        <div className="flex flex-wrap gap-2">
          <button
            type="button"
            onClick={onAtualizarSessoes}
            disabled={carregandoSessoes}
            className="rounded-md border border-slate-700 px-3 py-2 text-sm hover:bg-slate-800 disabled:cursor-wait disabled:opacity-50"
          >
            {carregandoSessoes ? 'Atualizando…' : 'Atualizar sessões'}
          </button>
          <button
            type="button"
            onClick={onVoltar}
            className="rounded-md bg-orange-500 px-3 py-2 text-sm font-semibold text-white hover:bg-orange-400"
          >
            Voltar ao dashboard
          </button>
        </div>
      </header>

      <div className="grid min-h-0 grid-cols-1 gap-6 p-6 lg:grid-cols-[320px_1fr]">
        <aside aria-label="Sessões" className="space-y-2">
          <h2 className="text-xs uppercase tracking-widest text-slate-500">
            {lista.length} {lista.length === 1 ? 'execução' : 'execuções'}
          </h2>

          {carregandoSessoes && sessoes === null && (
            <p className="text-sm text-slate-400">Carregando sessões...</p>
          )}
          {erroSessoes && (
            <p role="alert" className="rounded-lg border border-red-500/30 bg-red-500/10 p-3 text-sm text-red-300">
              {erroSessoes}
            </p>
          )}
          {sessoes && !erroSessoes && lista.length === 0 && (
            <p className="text-sm text-slate-400">Nenhuma sessão registrada.</p>
          )}

          {lista.map((s) => {
            const ativa = s.id_sessao === idSelecionada;
            return (
              <button
                key={s.id_sessao}
                type="button"
                onClick={() => setIdSelecionada(s.id_sessao)}
                aria-pressed={ativa}
                className={`w-full rounded-xl border px-4 py-3 text-left transition ${
                  ativa
                    ? 'border-orange-500 bg-white/5'
                    : 'border-slate-800 bg-white/[0.02] hover:border-slate-600'
                }`}
              >
                <div className="flex items-center justify-between gap-2">
                  <span className="font-semibold text-slate-100">{formatarData(s.data_inicio)}</span>
                  <span className="text-xs text-slate-500">#{s.id_sessao}</span>
                </div>
                <div className={`mt-1 text-sm ${s.desafio_cumprido ? 'text-emerald-400' : 'text-slate-300'}`}>
                  {s.status_conclusao} · {formatarDuracao(s.tempo_total_segundos)}
                </div>
                <div className="mt-1 text-xs text-slate-500">
                  {s.tipo_labirinto ?? 'Labirinto não informado'}
                  {s.numero_tentativa !== null ? ` · tentativa ${s.numero_tentativa}` : ''}
                  {s.consumo_bateria_mah !== null ? ` · ${s.consumo_bateria_mah.toFixed(1)} mAh` : ''}
                </div>
              </button>
            );
          })}
        </aside>

        <section
          aria-label="Trajeto e eventos da sessão"
          className="min-h-[320px] rounded-2xl border border-slate-800 bg-white/[0.02]"
        >
          {selecionada === undefined ? (
            <p className="p-6 text-sm text-slate-400">
              {lista.length === 0 ? 'Os trajetos e eventos serão exibidos aqui quando houver sessões registradas.' : 'Selecione uma execução para consultar o trajeto e seus eventos.'}
            </p>
          ) : (
            <>
              <section aria-label="Trajeto da sessão" className="border-b border-slate-800 p-4">
                <header className="mb-3 flex items-center justify-between gap-3">
                  <div>
                    <h2 className="font-semibold text-slate-100">Trajeto · sessão #{selecionada.id_sessao}</h2>
                    <p className="mt-1 text-xs text-slate-500">Coordenadas (x, y) e tempo relativo à largada</p>
                  </div>
                  <span className="text-xs text-slate-500">
                    {pontos?.chave === chavePontos ? `${pontos.dados.length} pontos` : 'Carregando…'}
                  </span>
                </header>
                {pontos?.chave !== chavePontos ? (
                  <p className="text-sm text-slate-400">Carregando trajeto...</p>
                ) : pontos.erro ? (
                  <p role="alert" className="text-sm text-red-300">{pontos.erro}</p>
                ) : pontos.dados.length === 0 ? (
                  <p className="text-sm text-slate-400">Esta sessão não contém pontos de percurso.</p>
                ) : (
                  <div className="max-h-56 overflow-auto rounded-lg border border-slate-800">
                    <table className="w-full text-left text-sm">
                      <thead className="sticky top-0 bg-slate-900 text-xs uppercase tracking-wider text-slate-500">
                        <tr>
                          <th className="px-3 py-2 font-medium">#</th>
                          <th className="px-3 py-2 font-medium">Tempo</th>
                          <th className="px-3 py-2 font-medium">Célula (x, y)</th>
                          <th className="px-3 py-2 font-medium">Orientação</th>
                        </tr>
                      </thead>
                      <tbody>
                        {pontos.dados.map((ponto) => (
                          <tr key={`${ponto.ordem}-${ponto.timestamp_ms}`} className="border-t border-slate-800">
                            <td className="px-3 py-2 text-slate-500">{ponto.ordem + 1}</td>
                            <td className="px-3 py-2 font-mono text-slate-400">{formatarTempo(ponto.timestamp_ms)}</td>
                            <td className="px-3 py-2 text-slate-200">({ponto.pos_x}, {ponto.pos_y})</td>
                            <td className="px-3 py-2 text-slate-300">{ponto.orientacao}</td>
                          </tr>
                        ))}
                      </tbody>
                    </table>
                  </div>
                )}
              </section>

              <header className="flex flex-wrap items-center justify-between gap-3 border-b border-slate-800 p-4">
                <div>
                  <h2 className="font-semibold text-slate-100">Eventos e diagnósticos</h2>
                  <p className="mt-1 text-xs text-slate-500">Sessão #{selecionada.id_sessao} · ordenados por timestamp</p>
                </div>
                <div className="flex gap-2" role="group" aria-label="Filtrar por nível">
                  {(['TODOS', ...NIVEIS] as Filtro[]).map((n) => (
                    <button
                      key={n}
                      type="button"
                      onClick={() => setFiltro(n)}
                      aria-pressed={filtro === n}
                      className={`rounded-full px-3 py-1 text-xs font-medium transition ${
                        filtro === n
                          ? 'bg-orange-500 text-white'
                          : 'bg-white/5 text-slate-300 hover:bg-white/10'
                      }`}
                    >
                      {n === 'TODOS' ? 'Todos' : n}
                    </button>
                  ))}
                </div>
              </header>
              <EventLog
                logs={logs?.chave === chaveLogs ? logs.dados : []}
                carregando={carregandoLogs}
                erro={logs?.chave === chaveLogs ? logs.erro : null}
              />
            </>
          )}
        </section>
      </div>
    </main>
  );
}
