import express from "express";
import cors from "cors";
import logsRouter from "./routes/logs.routes";

const app = express();

app.use(cors());
app.use(express.json());

app.get("/health", (_req, res) => {
  res.json({
    status: "ok",
    service: "micromouse-backend"
  });
});

app.use("/api/sessions", logsRouter);

export default app;