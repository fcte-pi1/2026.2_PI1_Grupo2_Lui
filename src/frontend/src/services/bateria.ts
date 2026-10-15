// Limites da bateria. Fonte: docs/4.2 - Projeto conceitual de energia.md
// (pack Li-ion/LiPo 2S de 7,4 V e 3300 mAh; alerta de descarga em ~6,4 V).
export const TENSAO_MAXIMA = 8.4;
export const TENSAO_CRITICA = 6.4;
export const TENSAO_MINIMA = 6.0;

export function bateriaCritica(tensao: number | null) {
  return tensao !== null && tensao < TENSAO_CRITICA;
}
