-- HU-11: metadados de sessão e persistência da sequência do percurso.
ALTER TABLE "SESSAO_NAVEGACAO"
  ALTER COLUMN "tempo_total_segundos" SET DEFAULT 0,
  ADD COLUMN "id_dispositivo" BIGINT,
  ADD COLUMN "tipo_labirinto" TEXT,
  ADD COLUMN "numero_tentativa" INTEGER,
  ADD COLUMN "desafio_cumprido" BOOLEAN NOT NULL DEFAULT FALSE,
  ADD COLUMN "consumo_bateria_mah" DOUBLE PRECISION;

CREATE UNIQUE INDEX "SESSAO_NAVEGACAO_id_dispositivo_key"
  ON "SESSAO_NAVEGACAO"("id_dispositivo");

CREATE TABLE "PONTO_PERCURSO" (
  "id_ponto" BIGSERIAL NOT NULL,
  "id_sessao" INTEGER NOT NULL,
  "ordem" INTEGER NOT NULL,
  "timestamp_ms" BIGINT NOT NULL,
  "pos_x" INTEGER NOT NULL,
  "pos_y" INTEGER NOT NULL,
  "orientacao" TEXT NOT NULL,
  CONSTRAINT "PONTO_PERCURSO_pkey" PRIMARY KEY ("id_ponto")
);

CREATE UNIQUE INDEX "PONTO_PERCURSO_id_sessao_ordem_key"
  ON "PONTO_PERCURSO"("id_sessao", "ordem");
CREATE INDEX "PONTO_PERCURSO_id_sessao_timestamp_ms_idx"
  ON "PONTO_PERCURSO"("id_sessao", "timestamp_ms");

ALTER TABLE "PONTO_PERCURSO"
  ADD CONSTRAINT "PONTO_PERCURSO_id_sessao_fkey"
  FOREIGN KEY ("id_sessao") REFERENCES "SESSAO_NAVEGACAO"("id_sessao")
  ON DELETE CASCADE ON UPDATE CASCADE;
