/**
 * HU-12 - Histórico de sessões, eventos e diagnósticos.
 *
 * Serviço de logs de eventos. Recebe o "banco" por parâmetro (injeção de
 * dependência) para ser testável sem PostgreSQL.
 */

export const NIVEIS_LOG = ["INFO", "WARN", "ERROR", "SUCCESS"] as const;
export type NivelLog = (typeof NIVEIS_LOG)[number];

export function isNivelLog(valor: unknown): valor is NivelLog {
  return typeof valor === "string" && (NIVEIS_LOG as readonly string[]).includes(valor);
}

/** Registro como vem do Prisma (timestamp_ms é BigInt). */
export interface LogEventoRow {
  id_log: number;
  id_sessao: number;
  timestamp_ms: bigint;
  tipo_evento: string;
  descricao: string;
}

/** Registro como vai para o frontend (JSON não serializa BigInt). */
export interface LogEventoDTO {
  id_log: number;
  timestamp_ms: number;
  tipo_evento: string;
  descricao: string;
}

/** Parte mínima do Prisma que o serviço usa. */
export interface LogEventoDb {
  // session_id do firmware é um identificador do dispositivo, não a PK do banco.
  sessaoNavegacao?: {
    findUnique(args: { where: { id_dispositivo: bigint } }): Promise<{ id_sessao: number } | null>;
  };
  logEvento: {
    create(args: {
      data: {
        id_sessao: number;
        timestamp_ms: bigint;
        tipo_evento: string;
        descricao: string;
      };
    }): Promise<LogEventoRow>;
    findMany(args: {
      where: { id_sessao: number; tipo_evento?: string };
      orderBy: { timestamp_ms: "asc" };
      take?: number;
    }): Promise<LogEventoRow[]>;
  };
}

export interface NovoLog {
  idSessaoDispositivo: number;
  nivel: string;
  descricao: string;
  timestampMs: number;
}

export class LogInvalidoError extends Error {}

function toDTO(row: LogEventoRow): LogEventoDTO {
  return {
    id_log: row.id_log,
    timestamp_ms: Number(row.timestamp_ms),
    tipo_evento: row.tipo_evento,
    descricao: row.descricao
  };
}

export function createLogEventoService(db: LogEventoDb) {
  return {
    async registrarLog(novo: NovoLog): Promise<LogEventoDTO> {
      if (!Number.isInteger(novo.idSessaoDispositivo) || novo.idSessaoDispositivo <= 0) {
        throw new LogInvalidoError("session_id inválido");
      }
      if (!isNivelLog(novo.nivel)) {
        throw new LogInvalidoError(`nível inválido (use ${NIVEIS_LOG.join(", ")})`);
      }
      if (typeof novo.descricao !== "string" || novo.descricao.trim() === "") {
        throw new LogInvalidoError("mensagem vazia");
      }
      if (!Number.isFinite(novo.timestampMs) || novo.timestampMs < 0) {
        throw new LogInvalidoError("t_ms inválido");
      }

      let idSessaoBanco = novo.idSessaoDispositivo;
      if (db.sessaoNavegacao) {
        const sessaoDispositivo = await db.sessaoNavegacao.findUnique({
          where: { id_dispositivo: BigInt(novo.idSessaoDispositivo) }
        });
        if (!sessaoDispositivo) {
          throw new LogInvalidoError("sessão do dispositivo inexistente; registre a sessão antes de enviar logs");
        }
        idSessaoBanco = sessaoDispositivo.id_sessao;
      }

      const criado = await db.logEvento.create({
        data: {
          id_sessao: idSessaoBanco,
          timestamp_ms: BigInt(Math.trunc(novo.timestampMs)),
          tipo_evento: novo.nivel,
          descricao: novo.descricao.trim()
        }
      });
      return toDTO(criado);
    },

    async listarLogs(
      idSessao: number,
      filtros: { nivel?: string; limite?: number } = {}
    ): Promise<LogEventoDTO[]> {
      if (filtros.nivel !== undefined && !isNivelLog(filtros.nivel)) {
        throw new LogInvalidoError(`nível inválido (use ${NIVEIS_LOG.join(", ")})`);
      }

      const linhas = await db.logEvento.findMany({
        where: {
          id_sessao: idSessao,
          ...(filtros.nivel ? { tipo_evento: filtros.nivel } : {})
        },
        orderBy: { timestamp_ms: "asc" },
        ...(filtros.limite ? { take: filtros.limite } : {})
      });
      return linhas.map(toDTO);
    }
  };
}

export type LogEventoService = ReturnType<typeof createLogEventoService>;
