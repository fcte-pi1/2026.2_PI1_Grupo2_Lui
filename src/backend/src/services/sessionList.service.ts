/**
 * HU-12 - Listagem mínima (somente leitura) de sessões para o histórico.
 * A escrita de sessões é responsabilidade da HU-11.
 */

export interface SessaoRow {
  id_sessao: number;
  data_inicio: Date;
  status_conclusao: string;
  tempo_total_segundos: number;
  tipo_labirinto: string | null;
  numero_tentativa: number | null;
  desafio_cumprido: boolean;
  consumo_bateria_mah: number | null;
}

export interface SessionListDb {
  sessaoNavegacao: {
    findMany(args: {
      where?: { status_conclusao?: string };
      orderBy: { data_inicio: "desc" };
      select: {
        id_sessao: true;
        data_inicio: true;
        status_conclusao: true;
        tempo_total_segundos: true;
        tipo_labirinto: true;
        numero_tentativa: true;
        desafio_cumprido: true;
        consumo_bateria_mah: true;
      };
    }): Promise<SessaoRow[]>;
  };
}

export function createSessionListService(db: SessionListDb) {
  return {
    listarSessoes(filtros: { status?: string } = {}): Promise<SessaoRow[]> {
      return db.sessaoNavegacao.findMany({
        ...(filtros.status ? { where: { status_conclusao: filtros.status } } : {}),
        orderBy: { data_inicio: "desc" },
        // Não expõe id_dispositivo (BigInt) na resposta JSON.
        select: {
          id_sessao: true,
          data_inicio: true,
          status_conclusao: true,
          tempo_total_segundos: true,
          tipo_labirinto: true,
          numero_tentativa: true,
          desafio_cumprido: true,
          consumo_bateria_mah: true,
        }
      });
    }
  };
}

export type SessionListService = ReturnType<typeof createSessionListService>;
