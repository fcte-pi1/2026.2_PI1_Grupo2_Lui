import "dotenv/config";
import { PrismaPg } from "@prisma/adapter-pg";

/** Valida a URL separadamente para permitir testes sem criar conexão com banco. */
export function requireDatabaseUrl(connectionString = process.env.DATABASE_URL): string {
  if (!connectionString) {
    throw new Error("DATABASE_URL não configurada");
  }
  return connectionString;
}

/** Cria o cliente sob demanda; importar módulos HTTP não inicializa o Prisma. */
export async function createPrismaClient(connectionString = process.env.DATABASE_URL) {
  const adapter = new PrismaPg({
    connectionString: requireDatabaseUrl(connectionString)
  });
  // A importação dinâmica permite testar a validação sem exigir artefatos gerados.
  const generatedClientModule = "../generated/prisma/client";
  const { PrismaClient } = await import(generatedClientModule);
  return new PrismaClient({ adapter });
}
