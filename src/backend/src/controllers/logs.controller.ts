import type { Request, Response } from "express";
import {
  LogInvalidoError,
  type LogEventoService
} from "../services/logEvento.service";
import type { SessionListService } from "../services/sessionList.service";

export function createLogsController(
  logs: LogEventoService,
  sessoes: SessionListService
) {
  return {
    async listarSessoes(req: Request, res: Response) {
      try {
        const status = typeof req.query.status === "string" ? req.query.status : undefined;
        res.json(await sessoes.listarSessoes({ status }));
      } catch (erro) {
        console.error("Erro ao listar sessões:", erro);
        res.status(500).json({ erro: "Erro interno ao listar sessões" });
      }
    },

    async listarLogs(req: Request, res: Response) {
      const idSessao = Number(req.params.id);
      if (!Number.isInteger(idSessao) || idSessao <= 0) {
        res.status(400).json({ erro: "id da sessão inválido" });
        return;
      }

      try {
        const nivel = typeof req.query.nivel === "string" ? req.query.nivel : undefined;
        res.json(await logs.listarLogs(idSessao, { nivel }));
      } catch (erro) {
        if (erro instanceof LogInvalidoError) {
          res.status(400).json({ erro: erro.message });
          return;
        }
        console.error("Erro ao listar logs:", erro);
        res.status(500).json({ erro: "Erro interno ao listar logs" });
      }
    }
  };
}
