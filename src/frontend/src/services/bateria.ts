// Limites da bateria. Valores de LiPo 2S/3S mudam: ajustar quando a equipe de
// Energia fechar a bateria usada no robô.
export const TENSAO_MAXIMA = 12.6;
export const TENSAO_CRITICA = 10.5;
export const TENSAO_MINIMA = 9.9;

export function bateriaCritica(tensao: number | null) {
  return tensao !== null && tensao < TENSAO_CRITICA;
}
