import { Router } from "express";
import { createLogsController } from "../controllers/logs.controller";
import type { LogEventoService } from "../services/logEvento.service";
import type { SessionListService } from "../services/sessionList.service";
import type { SessionNavigationService } from "../services/sessionNavigation.service";

export interface LogsRouterServices {
  logs: LogEventoService;
  sessoes: SessionListService;
  navegacao: SessionNavigationService;
}

/** Cria as rotas HTTP com serviços injetados; não exige PostgreSQL para testar. */
export function createLogsRouter(services: LogsRouterServices) {
  const router = Router();
  const controller = createLogsController(services.logs, services.sessoes, services.navegacao);

  // HU-11: início/finalização e caminho; HU-12: consulta cronológica de eventos.
  router.get("/", controller.listarSessoes);
  router.post("/", controller.iniciarSessao);
  router.post("/navigation", controller.finalizarSessao);
  router.get("/:id/navigation", controller.listarPontos);
  router.get("/:id/logs", controller.listarLogs);

  return router;
}
