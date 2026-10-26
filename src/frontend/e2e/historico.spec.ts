import { expect, test } from '@playwright/test';
import { instalarMocksApi } from './support/api-mocks';

test.describe('Histórico de execuções e filtros de eventos', () => {
  test.beforeEach(async ({ page }) => {
    await instalarMocksApi(page);
    await page.goto('/?fonte=simulador');
    await page.getByRole('button', { name: 'Histórico e diagnósticos' }).click();
  });

  test('consulta a sessão selecionada e apresenta os pontos do trajeto', async ({ page }) => {
    await expect(page.getByRole('heading', { name: 'Histórico de sessões e diagnósticos' })).toBeVisible();
    const detalhes = page.getByRole('region', { name: 'Trajeto e eventos da sessão' });
    await expect(detalhes.getByRole('heading', { name: 'Trajeto · sessão #702' })).toBeVisible();
    await expect(detalhes.getByText('3 pontos')).toBeVisible();
    await expect(detalhes.getByRole('cell', { name: '(0, 0)' })).toBeVisible();
    await expect(detalhes.getByRole('cell', { name: '(1, 1)' })).toBeVisible();
    await expect(detalhes.getByRole('heading', { name: 'Eventos e diagnósticos' })).toBeVisible();
  });

  test('filtra os eventos por severidade e volta a mostrar todos os níveis', async ({ page }) => {
    const detalhes = page.getByRole('region', { name: 'Trajeto e eventos da sessão' });
    await expect(detalhes.getByText('Execução iniciada para teste E2E')).toBeVisible();
    await expect(detalhes.getByText('Colisão simulada no corredor')).toBeVisible();

    await detalhes.getByRole('button', { name: 'ERROR', exact: true }).click();
    await expect(detalhes.getByText('Colisão simulada no corredor')).toBeVisible();
    await expect(detalhes.getByText('Execução iniciada para teste E2E')).toHaveCount(0);

    await detalhes.getByRole('button', { name: 'Todos', exact: true }).click();
    await expect(detalhes.getByText('Execução iniciada para teste E2E')).toBeVisible();
    await expect(detalhes.getByText('Objetivo alcançado')).toBeVisible();
  });

  test('permite voltar do histórico para o dashboard sem recarregar a aplicação', async ({ page }) => {
    await page.getByRole('button', { name: 'Voltar ao dashboard' }).click();
    await expect(page.getByRole('heading', { name: 'MicroMouse · Painel da corrida' })).toBeVisible();
  });
});
