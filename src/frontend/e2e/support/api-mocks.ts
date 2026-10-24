import type { Page, Route } from '@playwright/test';
import type { LogEvento, PontoPercurso, Sessao } from '../../src/services/api';
import { logsExemplo, pontosExemplo, sessoesExemplo } from './data';

interface MockOptions {
  sessoes?: Sessao[];
  logs?: LogEvento[];
  pontos?: PontoPercurso[];
  falharListagemSessoes?: boolean;
}

/** Simula os endpoints HTTP consumidos pelas telas, sem depender do PostgreSQL. */
export async function instalarMocksApi(page: Page, opcoes: MockOptions = {}): Promise<void> {
  const sessoes = opcoes.sessoes ?? sessoesExemplo;
  const logs = opcoes.logs ?? logsExemplo;
  const pontos = opcoes.pontos ?? pontosExemplo;

  await page.route('**/api/sessions**', async (route: Route) => {
    const url = new URL(route.request().url());
    const caminho = url.pathname.replace(/\/$/, '');

    if (caminho.endsWith('/api/sessions')) {
      if (opcoes.falharListagemSessoes) {
        await route.fulfill({ status: 503, contentType: 'application/json', json: { erro: 'Backend de teste indisponível' } });
        return;
      }
      await route.fulfill({ status: 200, contentType: 'application/json', json: sessoes });
      return;
    }

    const logsMatch = caminho.match(/\/api\/sessions\/(\d+)\/logs$/);
    if (logsMatch) {
      const id = Number(logsMatch[1]);
      const nivel = url.searchParams.get('nivel');
      const limite = Number(url.searchParams.get('limite') ?? 0);
      const recentes = url.searchParams.get('recentes') === 'true';
      let resultado = id === (sessoes[0]?.id_sessao ?? -1) ? [...logs] : [];
      if (nivel) resultado = resultado.filter((item) => item.tipo_evento === nivel);
      if (recentes) resultado.reverse();
      if (limite > 0) resultado = resultado.slice(0, limite);
      if (recentes) resultado.reverse();
      await route.fulfill({ status: 200, contentType: 'application/json', json: resultado });
      return;
    }

    const navigationMatch = caminho.match(/\/api\/sessions\/(\d+)\/navigation$/);
    if (navigationMatch) {
      const id = Number(navigationMatch[1]);
      await route.fulfill({
        status: 200,
        contentType: 'application/json',
        json: id === (sessoes[0]?.id_sessao ?? -1) ? pontos : [],
      });
      return;
    }

    await route.fulfill({ status: 404, contentType: 'application/json', json: { erro: 'Endpoint de teste não mapeado' } });
  });
}
