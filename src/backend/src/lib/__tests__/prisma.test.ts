import { describe, it } from "node:test";
import assert from "node:assert/strict";
import { createPrismaClient, requireDatabaseUrl } from "../prisma";

describe("Prisma configuration", () => {
  it("returns the configured connection string", () => {
    assert.equal(requireDatabaseUrl("postgresql://user:pass@localhost:5432/test"), "postgresql://user:pass@localhost:5432/test");
  });

  it("rejects missing and empty DATABASE_URL", () => {
    assert.throws(() => requireDatabaseUrl(""), /DATABASE_URL não configurada/);
  });

  it("fails before loading the generated client when configuration is missing", async () => {
    await assert.rejects(() => createPrismaClient(""), /DATABASE_URL não configurada/);
  });
});
