import type { Sessao } from "../services/api";

/** Formata uma data ISO usando o locale adotado pelo dashboard. */
export function formatarData(iso: string): string {
  const data = new Date(iso);
  if (Number.isNaN(data.getTime())) return "Data inválida";
  return data.toLocaleString("pt-BR", {
    day: "2-digit",
    month: "2-digit",
    year: "numeric",
    hour: "2-digit",
    minute: "2-digit",
  });
}

/** Formata milissegundos relativos ao início de uma sessão. */
export function formatarTempo(ms: number): string {
  const valor = Math.max(0, Math.trunc(ms));
  const minutos = Math.floor(valor / 60000);
  const segundos = Math.floor((valor % 60000) / 1000);
  const resto = valor % 1000;
  const dois = (n: number) => String(n).padStart(2, "0");
  return `${dois(minutos)}:${dois(segundos)}.${String(resto).padStart(3, "0")}`;
}

/** Formata duração em segundos como mm:ss, incluindo execuções longas. */
export function formatarDuracao(segundos: number): string {
  const valor = Math.max(0, Math.round(segundos));
  const min = Math.floor(valor / 60);
  const seg = valor % 60;
  return `${String(min).padStart(2, "0")}:${String(seg).padStart(2, "0")}`;
}

/** Usa o indicador estruturado da HU-11 e tolera registros antigos sem ele. */
export function sessaoConcluida(
  sessao: Pick<Sessao, "status_conclusao" | "desafio_cumprido">,
): boolean {
  if (typeof sessao.desafio_cumprido === "boolean") return sessao.desafio_cumprido;
  const status = sessao.status_conclusao.normalize("NFD").replace(/[\u0300-\u036f]/g, "").toUpperCase();
  return /CONCLUID|CHEGADA|SUCCESS/.test(status) && !/NAO|INTERROMPID|FALHA|ERRO/.test(status);
}
