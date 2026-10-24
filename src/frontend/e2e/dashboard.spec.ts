import { expect, test } from '@playwright/test';
import { instalarMocksApi } from './support/api-mocks';

test.describe('Dashboard e configuração do labirinto', () => {
  test.beforeEach(async ({ page }) => {
    await instalarMocksApi(page, { sessoes: [] });
    await page.goto('/?fonte=simulador');
  });

  test('abre o painel e oferece os três tamanhos de labirinto configuráveis', async ({ page }) => {
    await expect(page.getByRole('heading', { name: 'MicroMouse · Painel da corrida' })).toBeVisible();

    const seletorTamanho = page.getByRole('combobox').nth(1);
    await expect(seletorTamanho.locator('option')).toHaveText([
      'Labirinto 4x4',
      'Labirinto 8x4',
      'Labirinto 12x4',
    ]);

    for (const tamanho of ['4x4', '8x4', '12x4']) {
      await seletorTamanho.selectOption({ label: `Labirinto ${tamanho}` });
      await expect(page.getByRole('heading', { name: `Mapa do labirinto (${tamanho})` })).toBeVisible();
      await expect(page.getByRole('img', { name: `Mapa do labirinto ${tamanho.replace('x', ' por ')}` })).toBeVisible();
    }
  });

  test('permite reiniciar a simulação sem sair do dashboard', async ({ page }) => {
    await expect(page.getByRole('heading', { name: 'Status e telemetria' })).toBeVisible();
    await page.getByRole('button', { name: 'Reiniciar' }).click();
    await expect(page.getByRole('heading', { name: 'Mapa do labirinto (8x4)' })).toBeVisible();
  });

  test('informa quando o backend não está disponível para o resumo do histórico', async ({ page }) => {
    await page.unroute('**/api/sessions**');
    await instalarMocksApi(page, { sessoes: [], falharListagemSessoes: true });
    await page.getByRole('button', { name: 'Atualizar histórico' }).click();
    await expect(page.getByRole('alert')).toContainText('Não foi possível carregar as sessões');
  });
});
