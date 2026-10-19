import { Router } from "express";
import { prisma } from "../lib/prisma";
import {
  createLogEventoService,
  type LogEventoDb
} from "../services/logEvento.service";
import { createSessionListService, type SessionListDb } from "../services/sessionList.service";
import {
  createSessionNavigationService,
  type SessionNavigationDb
} from "../services/sessionNavigation.service";
import { createLogsController } from "../controllers/logs.controller";

// Instâncias compartilhadas: o gateway recebe o serviço no ponto de composição do servidor.
export const logEventoService = createLogEventoService(prisma as unknown as LogEventoDb);
const sessionListService = createSessionListService(prisma as unknown as SessionListDb);
const sessionNavigationService = createSessionNavigationService(prisma as unknown as SessionNavigationDb);
const controller = createLogsController(logEventoService, sessionListService, sessionNavigationService);

/** HU-11: início/finalização e caminho; HU-12: consulta cronológica de eventos. */
const router = Router();
router.get("/", controller.listarSessoes);
router.post("/", controller.iniciarSessao);
router.post("/navigation", controller.finalizarSessao);
router.get("/:id/navigation", controller.listarPontos);
router.get("/:id/logs", controller.listarLogs);

export default router;
