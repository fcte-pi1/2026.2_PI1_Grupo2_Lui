-- HU-09: sensores e bateria podem estar indisponíveis; ausência não significa zero.
ALTER TABLE "REGISTRO_TELEMETRIA"
  ALTER COLUMN "tensao_bateria" DROP NOT NULL,
  ALTER COLUMN "corrente_bateria" DROP NOT NULL,
  ADD COLUMN "distancia_frontal_mm" DOUBLE PRECISION,
  ADD COLUMN "distancia_esquerda_mm" DOUBLE PRECISION,
  ADD COLUMN "distancia_direita_mm" DOUBLE PRECISION,
  ADD COLUMN "erro_pid" DOUBLE PRECISION NOT NULL DEFAULT 0;
