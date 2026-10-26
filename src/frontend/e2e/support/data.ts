import type { LogEvento, PontoPercurso, Sessao } from '../../src/services/api';

export const sessaoExemplo: Sessao = {
  id_sessao: 702,
  data_inicio: '2026-10-24T12:00:00.000Z',
  status_conclusao: 'CONCLUIDO',
  tempo_total_segundos: 12.5,
  tipo_labirinto: 'Labirinto 4x4',
  numero_tentativa: 3,
  desafio_cumprido: true,
  consumo_bateria_mah: 12.4,
};

export const sessoesExemplo: Sessao[] = [sessaoExemplo];

export const logsExemplo: LogEvento[] = [
  { id_log: 9001, timestamp_ms: 100, tipo_evento: 'INFO', descricao: 'Execução iniciada para teste E2E' },
  { id_log: 9002, timestamp_ms: 250, tipo_evento: 'WARN', descricao: 'Parede detectada à frente' },
  { id_log: 9003, timestamp_ms: 400, tipo_evento: 'ERROR', descricao: 'Colisão simulada no corredor' },
  { id_log: 9004, timestamp_ms: 550, tipo_evento: 'SUCCESS', descricao: 'Objetivo alcançado' },
];

export const pontosExemplo: PontoPercurso[] = [
  { ordem: 0, timestamp_ms: 0, pos_x: 0, pos_y: 0, orientacao: 'NORTH' },
  { ordem: 1, timestamp_ms: 100, pos_x: 0, pos_y: 1, orientacao: 'NORTH' },
  { ordem: 2, timestamp_ms: 250, pos_x: 1, pos_y: 1, orientacao: 'EAST' },
];
