/**
 * HU-12 - Listagem mínima (somente leitura) de sessões para o histórico.
 * A escrita de sessões é responsabilidade da HU-11.
 */

export interface SessaoRow {
  id_sessao: number;
  data_inicio: Date;
  status_conclusao: string;
  tempo_total_segundos: number;
}

export interface SessionListDb {
  sessaoNavegacao: {
    findMany(args: {
      where?: { status_conclusao?: string };
      orderBy: { data_inicio: "desc" };
    }): Promise<SessaoRow[]>;
  };
}

export function createSessionListService(db: SessionListDb) {
  return {
    listarSessoes(filtros: { status?: string } = {}): Promise<SessaoRow[]> {
      return db.sessaoNavegacao.findMany({
        ...(filtros.status ? { where: { status_conclusao: filtros.status } } : {}),
        orderBy: { data_inicio: "desc" }
      });
    }
  };
}

export type SessionListService = ReturnType<typeof createSessionListService>;
