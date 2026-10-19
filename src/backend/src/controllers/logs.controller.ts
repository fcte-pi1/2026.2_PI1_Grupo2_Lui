import type { Request, Response } from "express";
import {
  LogInvalidoError,
  type LogEventoService
} from "../services/logEvento.service";
import type { SessionListService } from "../services/sessionList.service";
import {
  SessionNavigationError,
  type SessionNavigationService
} from "../services/sessionNavigation.service";

export function createLogsController(
  logs: LogEventoService,
  sessoes: SessionListService,
  navegacao: SessionNavigationService
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

    async iniciarSessao(req: Request, res: Response) {
      try {
        const sessao = await navegacao.iniciarSessao(req.body);
        res.status(201).json({
          id_sessao: sessao.id_sessao,
          device_session_id: sessao.id_dispositivo === null ? null : Number(sessao.id_dispositivo),
          data_inicio: sessao.data_inicio,
          status_conclusao: sessao.status_conclusao,
        });
      } catch (erro) {
        if (erro instanceof SessionNavigationError) {
          res.status(erro.statusCode).json({ erro: erro.message });
          return;
        }
        console.error("Erro ao iniciar sessão:", erro);
        res.status(500).json({ erro: "Erro interno ao iniciar sessão" });
      }
    },

    async finalizarSessao(req: Request, res: Response) {
      try {
        res.status(200).json(await navegacao.finalizarSessao(req.body));
      } catch (erro) {
        if (erro instanceof SessionNavigationError) {
          res.status(erro.statusCode).json({ erro: erro.message });
          return;
        }
        console.error("Erro ao persistir sessão:", erro);
        res.status(500).json({ erro: "Erro interno ao persistir sessão" });
      }
    },

    async listarPontos(req: Request, res: Response) {
      const idSessao = Number(req.params.id);
      if (!Number.isInteger(idSessao) || idSessao <= 0) {
        res.status(400).json({ erro: "id da sessão inválido" });
        return;
      }
      try {
        res.json(await navegacao.listarPontos(idSessao));
      } catch (erro) {
        if (erro instanceof SessionNavigationError) {
          res.status(erro.statusCode).json({ erro: erro.message });
          return;
        }
        console.error("Erro ao listar pontos do percurso:", erro);
        res.status(500).json({ erro: "Erro interno ao listar percurso" });
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
