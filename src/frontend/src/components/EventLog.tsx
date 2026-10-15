import type { LogEvento } from '../services/api'

const ESTILO_NIVEL: Record<string, string> = {
  INFO: 'bg-sky-500/15 text-sky-300 ring-sky-500/30',
  WARN: 'bg-amber-500/15 text-amber-300 ring-amber-500/30',
  ERROR: 'bg-red-500/15 text-red-300 ring-red-500/30',
  SUCCESS: 'bg-emerald-500/15 text-emerald-300 ring-emerald-500/30',
}

/** 83500 -> "01:23.500" (tempo desde o início da sessão) */
function formatarTempo(ms: number): string {
  const minutos = Math.floor(ms / 60000)
  const segundos = Math.floor((ms % 60000) / 1000)
  const resto = Math.floor(ms % 1000)
  const dois = (n: number) => String(n).padStart(2, '0')
  return `${dois(minutos)}:${dois(segundos)}.${String(resto).padStart(3, '0')}`
}

interface EventLogProps {
  logs: LogEvento[]
  carregando: boolean
  erro: string | null
}

export function EventLog({ logs, carregando, erro }: EventLogProps) {
  if (erro) {
    return <p className="p-6 text-sm text-red-300">{erro}</p>
  }
  if (carregando) {
    return <p className="p-6 text-sm text-gray-400">Carregando eventos...</p>
  }
  if (logs.length === 0) {
    return <p className="p-6 text-sm text-gray-400">Nenhum evento registrado.</p>
  }

  return (
    <table className="w-full text-left text-sm">
      <thead className="text-xs uppercase tracking-wider text-gray-500">
        <tr>
          <th className="px-4 py-2 font-medium">Tempo</th>
          <th className="px-4 py-2 font-medium">Nível</th>
          <th className="px-4 py-2 font-medium">Mensagem</th>
        </tr>
      </thead>
      <tbody>
        {logs.map((log) => (
          <tr key={log.id_log} className="border-t border-white/5">
            <td className="whitespace-nowrap px-4 py-2 font-mono text-gray-400">
              {formatarTempo(log.timestamp_ms)}
            </td>
            <td className="px-4 py-2">
              <span
                className={`rounded-full px-2 py-0.5 text-xs font-medium ring-1 ${
                  ESTILO_NIVEL[log.tipo_evento] ?? 'bg-gray-500/15 text-gray-300 ring-gray-500/30'
                }`}
              >
                {log.tipo_evento}
              </span>
            </td>
            <td className="px-4 py-2 text-gray-200">{log.descricao}</td>
          </tr>
        ))}
      </tbody>
    </table>
  )
}
