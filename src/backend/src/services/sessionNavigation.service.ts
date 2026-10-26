/**
 * HU-11 - ciclo de vida da sessão e persistência do caminho.
 * A rota usa um identificador do dispositivo (session_id do firmware) para
 * permitir logs em tempo real antes do fechamento da execução. O id_sessao do
 * banco continua sendo a chave pública usada pelo dashboard.
 */

export const TIPOS_LABIRINTO = ["4x4", "8x4", "12x4"] as const;
export type TipoLabirinto = (typeof TIPOS_LABIRINTO)[number];
export const MOTIVOS_FIM = ["CHEGADA", "PARADA_MANUAL", "ERRO"] as const;
export type MotivoFim = (typeof MOTIVOS_FIM)[number];
export const ORIENTACOES = ["NORTH", "EAST", "SOUTH", "WEST"] as const;
export type OrientacaoRota = (typeof ORIENTACOES)[number];

export interface SessaoNavigationRow {
  id_sessao: number;
  id_dispositivo: bigint | null;
  data_inicio: Date;
  status_conclusao: string;
  tempo_total_segundos: number;
  tipo_labirinto: string | null;
  numero_tentativa: number | null;
  desafio_cumprido: boolean;
  consumo_bateria_mah: number | null;
}

export interface PontoPercursoRow {
  id_ponto: bigint;
  id_sessao: number;
  ordem: number;
  timestamp_ms: bigint;
  pos_x: number;
  pos_y: number;
  orientacao: string;
}

interface SessaoCreateData {
  id_dispositivo: bigint;
  data_inicio: Date;
  status_conclusao: string;
  tempo_total_segundos: number;
  tipo_labirinto: string | null;
  numero_tentativa: number | null;
  desafio_cumprido: boolean;
  consumo_bateria_mah: number | null;
}

interface SessaoUpdateData {
  status_conclusao: string;
  tempo_total_segundos: number;
  desafio_cumprido: boolean;
  tipo_labirinto?: string | null;
  numero_tentativa?: number | null;
  consumo_bateria_mah?: number | null;
}

interface NovoPontoPercurso {
  id_sessao: number;
  ordem: number;
  timestamp_ms: bigint;
  pos_x: number;
  pos_y: number;
  orientacao: OrientacaoRota;
}

export interface SessionNavigationTx {
  sessaoNavegacao: {
    findUnique(args: { where: { id_dispositivo?: bigint; id_sessao?: number } }): Promise<SessaoNavigationRow | null>;
    create(args: { data: SessaoCreateData }): Promise<SessaoNavigationRow>;
    update(args: { where: { id_sessao: number }; data: SessaoUpdateData }): Promise<SessaoNavigationRow>;
  };
  pontoPercurso: {
    deleteMany(args: { where: { id_sessao: number } }): Promise<unknown>;
    createMany(args: { data: NovoPontoPercurso[] }): Promise<unknown>;
    findMany(args: {
      where: { id_sessao: number };
      orderBy: { ordem: "asc" };
    }): Promise<PontoPercursoRow[]>;
  };
}

export interface SessionNavigationDb {
  $transaction<T>(callback: (tx: SessionNavigationTx) => Promise<T>): Promise<T>;
}

export class SessionNavigationError extends Error {
  constructor(message: string, readonly statusCode: 400 | 404 | 409 = 400) {
    super(message);
    this.name = "SessionNavigationError";
  }
}

export interface IniciarSessaoInput {
  device_session_id: number;
  tipo_labirinto?: string;
  tentativa?: number;
}

interface PontoInput {
  t: number;
  x: number;
  y: number;
  heading: OrientacaoRota;
}

export interface FinalizarSessaoInput {
  session_id: number;
  tempo_total_ms: number;
  motivo: MotivoFim;
  stride: number;
  pontos: PontoInput[];
  tipo_labirinto?: string;
  tentativa?: number;
  desafio_cumprido?: boolean;
  consumo_bateria_mah?: number | null;
}

const isRecord = (value: unknown): value is Record<string, unknown> =>
  typeof value === "object" && value !== null && !Array.isArray(value);

function validarIdDispositivo(value: unknown): number {
  if (!Number.isInteger(value) || Number(value) < 1 || Number(value) > 0xffffffff) {
    throw new SessionNavigationError("device_session_id/session_id deve ser um inteiro positivo de 32 bits");
  }
  return Number(value);
}

function validarTipoLabirinto(value: unknown): TipoLabirinto | undefined {
  if (value === undefined || value === null || value === "" || value === "desconhecido") return undefined;
  if (typeof value !== "string" || !(TIPOS_LABIRINTO as readonly string[]).includes(value)) {
    throw new SessionNavigationError(`tipo_labirinto inválido (use ${TIPOS_LABIRINTO.join(", ")})`);
  }
  return value as TipoLabirinto;
}

function validarTentativa(value: unknown): number | undefined {
  if (value === undefined || value === null) return undefined;
  if (!Number.isInteger(value) || Number(value) < 1 || Number(value) > 65535) {
    throw new SessionNavigationError("tentativa deve ser um inteiro entre 1 e 65535");
  }
  return Number(value);
}

function validarPonto(value: unknown, index: number): PontoInput {
  if (!isRecord(value)) throw new SessionNavigationError(`pontos[${index}] inválido`);
  const { t, x, y, heading } = value;
  if (!Number.isInteger(t) || Number(t) < 0 || Number(t) > Number.MAX_SAFE_INTEGER) {
    throw new SessionNavigationError(`pontos[${index}].t inválido`);
  }
  if (!Number.isInteger(x) || Number(x) < 0 || Number(x) > 32767 ||
      !Number.isInteger(y) || Number(y) < 0 || Number(y) > 32767) {
    throw new SessionNavigationError(`coordenadas inválidas em pontos[${index}]`);
  }
  const aliases: Record<string, OrientacaoRota> = {
    N: "NORTH", NORTH: "NORTH",
    E: "EAST", EAST: "EAST",
    S: "SOUTH", SOUTH: "SOUTH",
    W: "WEST", WEST: "WEST",
  };
  if (typeof heading !== "string" || !aliases[heading.toUpperCase()]) {
    throw new SessionNavigationError(`orientação inválida em pontos[${index}]`);
  }
  return { t: Number(t), x: Number(x), y: Number(y), heading: aliases[heading.toUpperCase()] };
}

function validarFinalizacao(input: unknown): FinalizarSessaoInput {
  if (!isRecord(input)) throw new SessionNavigationError("corpo da sessão inválido");
  const session_id = validarIdDispositivo(input.session_id);
  const tempo = input.tempo_total_ms;
  if (!Number.isInteger(tempo) || Number(tempo) < 0 || Number(tempo) > 0xffffffff) {
    throw new SessionNavigationError("tempo_total_ms inválido");
  }
  if (typeof input.motivo !== "string" || !(MOTIVOS_FIM as readonly string[]).includes(input.motivo)) {
    throw new SessionNavigationError(`motivo inválido (use ${MOTIVOS_FIM.join(", ")})`);
  }
  if (!Number.isInteger(input.stride) || Number(input.stride) < 1) {
    throw new SessionNavigationError("stride deve ser um inteiro positivo");
  }
  if (!Array.isArray(input.pontos) || input.pontos.length === 0 || input.pontos.length > 100_000) {
    throw new SessionNavigationError("pontos deve conter entre 1 e 100000 posições");
  }
  const pontos = input.pontos.map(validarPonto);
  for (let i = 1; i < pontos.length; i++) {
    if (pontos[i].t < pontos[i - 1].t) {
      throw new SessionNavigationError("timestamps dos pontos devem estar em ordem não decrescente");
    }
  }
  const tipo = validarTipoLabirinto(input.tipo_labirinto);
  const tentativa = validarTentativa(input.tentativa);
  if (input.desafio_cumprido !== undefined && typeof input.desafio_cumprido !== "boolean") {
    throw new SessionNavigationError("desafio_cumprido deve ser booleano");
  }
  const bateria = input.consumo_bateria_mah;
  if (bateria !== undefined && bateria !== null &&
      (typeof bateria !== "number" || !Number.isFinite(bateria) || bateria < 0)) {
    throw new SessionNavigationError("consumo_bateria_mah deve ser um número não negativo ou null");
  }
  return {
    session_id,
    tempo_total_ms: Number(tempo),
    motivo: input.motivo as MotivoFim,
    stride: Number(input.stride),
    pontos,
    ...(tipo ? { tipo_labirinto: tipo } : {}),
    ...(tentativa !== undefined ? { tentativa } : {}),
    ...(input.desafio_cumprido !== undefined ? { desafio_cumprido: input.desafio_cumprido as boolean } : {}),
    ...(bateria !== undefined ? { consumo_bateria_mah: bateria as number | null } : {}),
  };
}

const statusFinal = (motivo: MotivoFim): string => {
  if (motivo === "CHEGADA") return "CONCLUIDO";
  if (motivo === "PARADA_MANUAL") return "INTERROMPIDO";
  return "INTERROMPIDO_POR_FALHA";
};

export function createSessionNavigationService(db: SessionNavigationDb) {
  return {
    async iniciarSessao(input: unknown): Promise<SessaoNavigationRow> {
      if (!isRecord(input)) throw new SessionNavigationError("corpo da sessão inválido");
      const deviceId = validarIdDispositivo(input.device_session_id);
      const tipo = validarTipoLabirinto(input.tipo_labirinto);
      const tentativa = validarTentativa(input.tentativa);

      return db.$transaction(async (tx) => {
        const existente = await tx.sessaoNavegacao.findUnique({ where: { id_dispositivo: BigInt(deviceId) } });
        if (existente) {
          if (existente.status_conclusao !== "EM_ANDAMENTO") {
            throw new SessionNavigationError("o identificador do dispositivo já pertence a uma sessão encerrada", 409);
          }
          return existente; // POST repetido é idempotente.
        }
        return tx.sessaoNavegacao.create({
          data: {
            id_dispositivo: BigInt(deviceId),
            data_inicio: new Date(),
            status_conclusao: "EM_ANDAMENTO",
            tempo_total_segundos: 0,
            tipo_labirinto: tipo ?? null,
            numero_tentativa: tentativa ?? null,
            desafio_cumprido: false,
            consumo_bateria_mah: null,
          },
        });
      });
    },

    async finalizarSessao(raw: unknown): Promise<{ id_sessao: number; status_conclusao: string; pontos_registrados: number }> {
      const input = validarFinalizacao(raw);
      return db.$transaction(async (tx) => {
        const existente = await tx.sessaoNavegacao.findUnique({ where: { id_dispositivo: BigInt(input.session_id) } });
        const concluido = input.desafio_cumprido ?? input.motivo === "CHEGADA";
        const status = statusFinal(input.motivo);
        const dataInicio = existente?.data_inicio ?? new Date(Date.now() - input.tempo_total_ms);
        const sessao = existente
          ? await tx.sessaoNavegacao.update({
              where: { id_sessao: existente.id_sessao },
              data: {
                status_conclusao: status,
                tempo_total_segundos: input.tempo_total_ms / 1000,
                desafio_cumprido: concluido,
                ...(input.tipo_labirinto !== undefined ? { tipo_labirinto: input.tipo_labirinto } : {}),
                ...(input.tentativa !== undefined ? { numero_tentativa: input.tentativa } : {}),
                ...(input.consumo_bateria_mah !== undefined ? { consumo_bateria_mah: input.consumo_bateria_mah } : {}),
              },
            })
          : await tx.sessaoNavegacao.create({
              data: {
                id_dispositivo: BigInt(input.session_id),
                data_inicio: dataInicio,
                status_conclusao: status,
                tempo_total_segundos: input.tempo_total_ms / 1000,
                tipo_labirinto: input.tipo_labirinto ?? null,
                numero_tentativa: input.tentativa ?? null,
                desafio_cumprido: concluido,
                consumo_bateria_mah: input.consumo_bateria_mah ?? null,
              },
            });

        // Substituir a sequência numa transação torna o retry do POST idempotente.
        await tx.pontoPercurso.deleteMany({ where: { id_sessao: sessao.id_sessao } });
        await tx.pontoPercurso.createMany({
          data: input.pontos.map((ponto, ordem) => ({
            id_sessao: sessao.id_sessao,
            ordem,
            timestamp_ms: BigInt(ponto.t),
            pos_x: ponto.x,
            pos_y: ponto.y,
            orientacao: ponto.heading,
          })),
        });
        return { id_sessao: sessao.id_sessao, status_conclusao: sessao.status_conclusao, pontos_registrados: input.pontos.length };
      });
    },

    async listarPontos(idSessao: number): Promise<Array<{
      ordem: number;
      timestamp_ms: number;
      pos_x: number;
      pos_y: number;
      orientacao: string;
    }>> {
      if (!Number.isInteger(idSessao) || idSessao <= 0) {
        throw new SessionNavigationError("id da sessão inválido");
      }
      return db.$transaction(async (tx) => {
        const sessao = await tx.sessaoNavegacao.findUnique({ where: { id_sessao: idSessao } });
        if (!sessao) throw new SessionNavigationError("sessão não encontrada", 404);
        const pontos = await tx.pontoPercurso.findMany({
          where: { id_sessao: idSessao },
          orderBy: { ordem: "asc" },
        });
        return pontos.map((ponto) => ({
          ordem: ponto.ordem,
          timestamp_ms: Number(ponto.timestamp_ms),
          pos_x: ponto.pos_x,
          pos_y: ponto.pos_y,
          orientacao: ponto.orientacao,
        }));
      });
    },
  };
}

export type SessionNavigationService = ReturnType<typeof createSessionNavigationService>;
