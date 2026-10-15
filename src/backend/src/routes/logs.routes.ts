import { Router } from "express";
import { prisma } from "../lib/prisma";
import { createLogEventoService } from "../services/logEvento.service";
import { createSessionListService } from "../services/sessionList.service";
import { createLogsController } from "../controllers/logs.controller";

export const logEventoService = createLogEventoService(prisma);
const sessionListService = createSessionListService(prisma);
const controller = createLogsController(logEventoService, sessionListService);

/** Montado em /api/sessions (HU-12). Rotas de escrita de sessão são da HU-11. */
const router = Router();

router.get("/", controller.listarSessoes);
router.get("/:id/logs", controller.listarLogs);

export default router;
