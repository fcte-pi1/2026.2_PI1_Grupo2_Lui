import express, { type Router } from "express";
import cors from "cors";

/** Constrói a aplicação HTTP. O roteador é opcional para isolar /health em testes. */
export function createApp(logsRouter?: Router) {
  const app = express();
  app.use(cors());
  app.use(express.json());

  app.get("/health", (_req, res) => {
    res.json({
      status: "ok",
      service: "micromouse-backend"
    });
  });

  if (logsRouter) app.use("/api/sessions", logsRouter);
  return app;
}

export default createApp;
