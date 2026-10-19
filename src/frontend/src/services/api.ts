/**
 * Cliente HTTP compartilhado pelas telas de dashboard e histórico (HU-11/HU-12).
 * Rotas de sessão em /api/sessions; o WebSocket de telemetria é independente.
 */
export const NIVEIS = ["INFO", "WARN", "ERROR", "SUCCESS"] as const;
export type Nivel = (typeof NIVEIS)[number];

export interface Sessao {
  id_sessao: number;
  data_inicio: string;
  status_conclusao: string;
  tempo_total_segundos: number;
  tipo_labirinto: string | null;
  numero_tentativa: number | null;
  desafio_cumprido: boolean;
  consumo_bateria_mah: number | null;
}

export interface LogEvento {
  id_log: number;
  /** Milissegundos desde o início da sessão. */
  timestamp_ms: number;
  tipo_evento: string;
  descricao: string;
}

export interface PontoPercurso {
  ordem: number;
  timestamp_ms: number;
  pos_x: number;
  pos_y: number;
  orientacao: string;
}

const API_URL = import.meta.env.VITE_API_URL ?? "http://localhost:3000";

async function get<T>(caminho: string): Promise<T> {
  const resposta = await fetch(`${API_URL}${caminho}`);
  if (!resposta.ok) {
    let detalhe = "";
    try {
      const corpo: unknown = await resposta.json();
      if (typeof corpo === "object" && corpo !== null && "erro" in corpo && typeof corpo.erro === "string") {
        detalhe = `: ${corpo.erro}`;
      }
    } catch {
      // A API pode responder sem JSON; o status HTTP já é suficiente para o erro.
    }
    throw new Error(`Falha na requisição (${resposta.status})${detalhe}`);
  }
  return (await resposta.json()) as T;
}

export function getSessions(): Promise<Sessao[]> {
  return get<Sessao[]>("/api/sessions");
}

export interface FiltrosLog {
  limite?: number;
  recentes?: boolean;
}

export function getSessionLogs(
  idSessao: number,
  nivel?: Nivel,
  filtros: FiltrosLog = {},
): Promise<LogEvento[]> {
  const query = new URLSearchParams();
  if (nivel) query.set("nivel", nivel);
  if (filtros.limite !== undefined) query.set("limite", String(filtros.limite));
  if (filtros.recentes) query.set("recentes", "true");
  const parametros = query.toString();
  const sufixo = parametros ? `?${parametros}` : "";
  return get<LogEvento[]>(`/api/sessions/${idSessao}/logs${sufixo}`);
}

export function getSessionNavigation(idSessao: number): Promise<PontoPercurso[]> {
  return get<PontoPercurso[]>(`/api/sessions/${idSessao}/navigation`);
}
