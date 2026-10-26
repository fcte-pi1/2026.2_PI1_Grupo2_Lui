import { expect, test } from '@playwright/test';
import { instalarMocksApi } from './support/api-mocks';

test.describe('Corrida simulada', () => {
  test('apresenta mapa, posição do robô e métricas de telemetria sem hardware', async ({ page }) => {
    await instalarMocksApi(page, { sessoes: [] });
    await page.goto('/?fonte=simulador');

    await expect(page.getByRole('heading', { name: 'Mapa do labirinto (8x4)' })).toBeVisible();
    await expect(page.getByText(/^Robô na célula \(/)).toBeVisible();
    await expect(page.getByText('Bateria', { exact: true })).toBeVisible();
    await expect(page.getByText(/\d+\.\d{2} V/).first()).toBeVisible();
    await expect(page.getByText('Roda esquerda', { exact: true })).toBeVisible();
    await expect(page.getByText('Roda direita', { exact: true })).toBeVisible();
    await expect(page.getByText('Erro do controle PID')).toBeVisible();
  });

  test('permite alternar para a fonte de simulação usando o seletor do cabeçalho', async ({ page }) => {
    await instalarMocksApi(page, { sessoes: [] });
    await page.goto('/');
    await page.getByRole('combobox').nth(0).selectOption('simulador');

    await expect(page.getByText(/^Robô na célula \(/)).toBeVisible();
    await expect(page.getByText(/\d+\.\d{2} V/).first()).toBeVisible();
  });
});
