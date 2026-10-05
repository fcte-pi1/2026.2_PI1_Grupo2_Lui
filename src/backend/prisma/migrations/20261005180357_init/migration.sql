-- CreateTable
CREATE TABLE "SESSAO_NAVEGACAO" (
    "id_sessao" SERIAL NOT NULL,
    "data_inicio" TIMESTAMP(3) NOT NULL DEFAULT CURRENT_TIMESTAMP,
    "status_conclusao" TEXT NOT NULL,
    "tempo_total_segundos" DOUBLE PRECISION NOT NULL,

    CONSTRAINT "SESSAO_NAVEGACAO_pkey" PRIMARY KEY ("id_sessao")
);

-- CreateTable
CREATE TABLE "CELULA_LABIRINTO" (
    "id_celula" SERIAL NOT NULL,
    "id_sessao" INTEGER NOT NULL,
    "pos_x" INTEGER NOT NULL,
    "pos_y" INTEGER NOT NULL,
    "parede_norte" BOOLEAN NOT NULL,
    "parede_sul" BOOLEAN NOT NULL,
    "parede_leste" BOOLEAN NOT NULL,
    "parede_oeste" BOOLEAN NOT NULL,
    "visitada" BOOLEAN NOT NULL,

    CONSTRAINT "CELULA_LABIRINTO_pkey" PRIMARY KEY ("id_celula")
);

-- CreateTable
CREATE TABLE "REGISTRO_TELEMETRIA" (
    "id_telemetria" BIGSERIAL NOT NULL,
    "id_sessao" INTEGER NOT NULL,
    "timestamp_ms" BIGINT NOT NULL,
    "tensao_bateria" DOUBLE PRECISION NOT NULL,
    "corrente_bateria" DOUBLE PRECISION NOT NULL,
    "velocidade_esq" DOUBLE PRECISION NOT NULL,
    "velocidade_dir" DOUBLE PRECISION NOT NULL,
    "angulo_orientacao" DOUBLE PRECISION NOT NULL,

    CONSTRAINT "REGISTRO_TELEMETRIA_pkey" PRIMARY KEY ("id_telemetria")
);

-- CreateTable
CREATE TABLE "LOG_EVENTO" (
    "id_log" SERIAL NOT NULL,
    "id_sessao" INTEGER NOT NULL,
    "timestamp_ms" BIGINT NOT NULL,
    "tipo_evento" TEXT NOT NULL,
    "descricao" TEXT NOT NULL,

    CONSTRAINT "LOG_EVENTO_pkey" PRIMARY KEY ("id_log")
);

-- AddForeignKey
ALTER TABLE "CELULA_LABIRINTO" ADD CONSTRAINT "CELULA_LABIRINTO_id_sessao_fkey" FOREIGN KEY ("id_sessao") REFERENCES "SESSAO_NAVEGACAO"("id_sessao") ON DELETE RESTRICT ON UPDATE CASCADE;

-- AddForeignKey
ALTER TABLE "REGISTRO_TELEMETRIA" ADD CONSTRAINT "REGISTRO_TELEMETRIA_id_sessao_fkey" FOREIGN KEY ("id_sessao") REFERENCES "SESSAO_NAVEGACAO"("id_sessao") ON DELETE RESTRICT ON UPDATE CASCADE;

-- AddForeignKey
ALTER TABLE "LOG_EVENTO" ADD CONSTRAINT "LOG_EVENTO_id_sessao_fkey" FOREIGN KEY ("id_sessao") REFERENCES "SESSAO_NAVEGACAO"("id_sessao") ON DELETE RESTRICT ON UPDATE CASCADE;
