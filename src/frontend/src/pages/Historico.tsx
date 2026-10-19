import { useEffect, useState } from 'react'
import { EventLog } from '../components/EventLog'
import {
  getSessionLogs,
  getSessionNavigation,
  getSessions,
  NIVEIS,
  type LogEvento,
  type Nivel,
  type PontoPercurso,
  type Sessao,
} from '../services/api'

type Filtro = Nivel | 'TODOS'

interface Resultado<T> {
  chave: string
  dados: T
  erro: string | null
}

const mensagemErro = (e: unknown) =>
  e instanceof Error ? e.message : 'Erro ao comunicar com o servidor'

function formatarData(iso: string): string {
  return new Date(iso).toLocaleString('pt-BR', {
    day: '2-digit',
    month: '2-digit',
    hour: '2-digit',
    minute: '2-digit',
  })
}

function formatarTempo(ms: number): string {
  const minutos = Math.floor(ms / 60000)
  const segundos = Math.floor((ms % 60000) / 1000)
  const resto = Math.floor(ms % 1000)
  const dois = (n: number) => String(n).padStart(2, '0')
  return `${dois(minutos)}:${dois(segundos)}.${String(resto).padStart(3, '0')}`
}

function formatarDuracao(segundos: number): string {
  const min = Math.floor(segundos / 60)
  const seg = Math.round(segundos % 60)
  return `${String(min).padStart(2, '0')}:${String(seg).padStart(2, '0')}`
}

/** O texto exato de status_conclusao vem da HU-11; aqui só destacamos sucesso. */
const concluida = (s: Sessao) =>
  !/n[ãa]o/i.test(s.status_conclusao) && /conclu|chegada|success/i.test(s.status_conclusao)

export default function Historico() {
  const [sessoes, setSessoes] = useState<Resultado<Sessao[]> | null>(null)
  const [idSelecionada, setIdSelecionada] = useState<number | null>(null)
  const [filtro, setFiltro] = useState<Filtro>('TODOS')
  const [logs, setLogs] = useState<Resultado<LogEvento[]> | null>(null)
  const [pontos, setPontos] = useState<Resultado<PontoPercurso[]> | null>(null)

  useEffect(() => {
    getSessions()
      .then((dados) => setSessoes({ chave: 'sessoes', dados, erro: null }))
      .catch((e) => setSessoes({ chave: 'sessoes', dados: [], erro: mensagemErro(e) }))
  }, [])

  const chaveLogs = `${idSelecionada}:${filtro}`
  const chavePontos = idSelecionada === null ? '' : String(idSelecionada)

  useEffect(() => {
    if (idSelecionada === null) return
    let cancelado = false
    const nivel = filtro === 'TODOS' ? undefined : filtro
    getSessionLogs(idSelecionada, nivel)
      .then((dados) => !cancelado && setLogs({ chave: chaveLogs, dados, erro: null }))
      .catch((e) => !cancelado && setLogs({ chave: chaveLogs, dados: [], erro: mensagemErro(e) }))
    return () => {
      cancelado = true
    }
  }, [idSelecionada, filtro, chaveLogs])

  useEffect(() => {
    if (idSelecionada === null) return
    let cancelado = false
    getSessionNavigation(idSelecionada)
      .then((dados) => !cancelado && setPontos({ chave: chavePontos, dados, erro: null }))
      .catch((e) => !cancelado && setPontos({ chave: chavePontos, dados: [], erro: mensagemErro(e) }))
    return () => { cancelado = true }
  }, [idSelecionada, chavePontos])

  const lista = sessoes?.dados ?? []
  const selecionada = lista.find((s) => s.id_sessao === idSelecionada)
  // Carregando = ainda não chegou o resultado da combinação sessão+filtro atual.
  const carregandoLogs = idSelecionada !== null && logs?.chave !== chaveLogs

  return (
    <div className="grid min-h-0 flex-1 grid-cols-1 gap-6 p-6 lg:grid-cols-[320px_1fr]">
      <aside aria-label="Sessões" className="space-y-2">
        <h2 className="text-xs uppercase tracking-widest text-gray-500">
          {lista.length} {lista.length === 1 ? 'execução' : 'execuções'}
        </h2>

        {sessoes === null && <p className="text-sm text-gray-400">Carregando sessões...</p>}
        {sessoes?.erro && <p className="text-sm text-red-300">{sessoes.erro}</p>}
        {sessoes && !sessoes.erro && lista.length === 0 && (
          <p className="text-sm text-gray-400">Nenhuma sessão registrada.</p>
        )}

        {lista.map((s) => {
          const ativa = s.id_sessao === idSelecionada
          return (
            <button
              key={s.id_sessao}
              type="button"
              onClick={() => setIdSelecionada(s.id_sessao)}
              className={`w-full rounded-xl border px-4 py-3 text-left transition ${
                ativa
                  ? 'border-orange-500 bg-white/5'
                  : 'border-white/10 bg-white/[0.02] hover:border-white/25'
              }`}
            >
              <div className="flex items-center justify-between">
                <span className="font-semibold text-gray-100">{formatarData(s.data_inicio)}</span>
                <span className="text-xs text-gray-500">#{s.id_sessao}</span>
              </div>
              <div
                className={`mt-1 text-sm ${concluida(s) ? 'text-emerald-400' : 'text-red-300'}`}
              >
                {s.status_conclusao} · {formatarDuracao(s.tempo_total_segundos)}
              </div>
              <div className="mt-1 text-xs text-gray-500">
                {s.tipo_labirinto ?? 'Labirinto não informado'}
                {s.numero_tentativa !== null ? ` · tentativa ${s.numero_tentativa}` : ''}
                {s.consumo_bateria_mah !== null ? ` · ${s.consumo_bateria_mah.toFixed(1)} mAh` : ''}
              </div>
            </button>
          )
        })}
      </aside>

      <section
        aria-label="Eventos da sessão"
        className="min-h-[320px] rounded-2xl border border-white/10 bg-white/[0.02]"
      >
        {selecionada === undefined ? (
          <p className="p-6 text-sm text-gray-400">
            Selecione uma execução para ver o histórico de eventos.
          </p>
        ) : (
          <>
            <section aria-label="Trajeto da sessão" className="border-b border-white/10 p-4">
              <header className="mb-3 flex items-center justify-between gap-3">
                <h2 className="font-semibold text-gray-100">Trajeto · {pontos?.chave === chavePontos ? pontos.dados.length : '…'} pontos</h2>
                <span className="text-xs text-gray-500">coordenadas (x, y) e tempo relativo</span>
              </header>
              {pontos?.chave !== chavePontos ? (
                <p className="text-sm text-gray-400">Carregando trajeto...</p>
              ) : pontos.erro ? (
                <p className="text-sm text-red-300">{pontos.erro}</p>
              ) : pontos.dados.length === 0 ? (
                <p className="text-sm text-gray-400">Esta sessão não contém pontos de percurso.</p>
              ) : (
                <div className="max-h-56 overflow-auto rounded-lg border border-white/5">
                  <table className="w-full text-left text-sm">
                    <thead className="sticky top-0 bg-[#151821] text-xs uppercase tracking-wider text-gray-500">
                      <tr>
                        <th className="px-3 py-2 font-medium">#</th>
                        <th className="px-3 py-2 font-medium">Tempo</th>
                        <th className="px-3 py-2 font-medium">Célula (x, y)</th>
                        <th className="px-3 py-2 font-medium">Orientação</th>
                      </tr>
                    </thead>
                    <tbody>
                      {pontos.dados.map((ponto) => (
                        <tr key={`${ponto.ordem}-${ponto.timestamp_ms}`} className="border-t border-white/5">
                          <td className="px-3 py-2 text-gray-500">{ponto.ordem + 1}</td>
                          <td className="px-3 py-2 font-mono text-gray-400">{formatarTempo(ponto.timestamp_ms)}</td>
                          <td className="px-3 py-2 text-gray-200">({ponto.pos_x}, {ponto.pos_y})</td>
                          <td className="px-3 py-2 text-gray-300">{ponto.orientacao}</td>
                        </tr>
                      ))}
                    </tbody>
                  </table>
                </div>
              )}
            </section>
            <header className="flex flex-wrap items-center justify-between gap-3 border-b border-white/10 p-4">
              <h2 className="font-semibold text-gray-100">
                Eventos · sessão #{selecionada.id_sessao}
              </h2>
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
                        : 'bg-white/5 text-gray-300 hover:bg-white/10'
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
  )
}
