/**
 * HU-12 - Cliente HTTP do histórico de sessões e logs.
 * Rotas em /api/sessions (backend).
 */

export const NIVEIS = ['INFO', 'WARN', 'ERROR', 'SUCCESS'] as const
export type Nivel = (typeof NIVEIS)[number]

export interface Sessao {
  id_sessao: number
  data_inicio: string
  status_conclusao: string
  tempo_total_segundos: number
}

export interface LogEvento {
  id_log: number
  /** ms desde o início da sessão */
  timestamp_ms: number
  tipo_evento: string
  descricao: string
}

const API_URL = import.meta.env.VITE_API_URL ?? 'http://localhost:3000'

async function get<T>(caminho: string): Promise<T> {
  const resposta = await fetch(`${API_URL}${caminho}`)
  if (!resposta.ok) {
    throw new Error(`Falha na requisição (${resposta.status})`)
  }
  return (await resposta.json()) as T
}

export function getSessions(): Promise<Sessao[]> {
  return get<Sessao[]>('/api/sessions')
}

export function getSessionLogs(idSessao: number, nivel?: Nivel): Promise<LogEvento[]> {
  const query = nivel ? `?nivel=${nivel}` : ''
  return get<LogEvento[]>(`/api/sessions/${idSessao}/logs${query}`)
}
