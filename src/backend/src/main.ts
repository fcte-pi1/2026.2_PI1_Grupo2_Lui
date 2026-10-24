import { startServer } from "./server";

void startServer().catch((error: unknown) => {
  console.error("Falha ao iniciar o backend:", error);
  process.exitCode = 1;
});
