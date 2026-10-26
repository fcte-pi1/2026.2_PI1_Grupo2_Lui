import { expect, test } from '@playwright/test';
import { instalarMocksApi } from './support/api-mocks';

test('aplica mensagens recebidas pelo WebSocket aos indicadores e ao mapa', async ({ page }) => {
  await instalarMocksApi(page, { sessoes: [] });
  await page.routeWebSocket('ws://localhost:3000/ws?role=dashboard', (socket) => {
    // Mensagens determinísticas: o teste valida o consumidor WebSocket real do frontend.
    setTimeout(() => {
      socket.send(JSON.stringify({ type: 'maze', width: 8, height: 4 }));
      socket.send(JSON.stringify({
        type: 'cell', x: 3, y: 1, visited: true,
        walls: { north: true, south: false, east: true, west: false },
      }));
      socket.send(JSON.stringify({ type: 'position', x: 3, y: 1, heading: 'N' }));
      socket.send(JSON.stringify({
        type: 'telemetry', battery_voltage: 11.2, pid_error: 0.04,
        wheels_speed: { left: 1.2, right: 1.18 }, robot_state: 'MAPPING',
        timestamp: 1_790_000_000_000,
      }));
    }, 100);
  });

  await page.goto('/');
  await expect(page.getByRole('heading', { name: 'Mapa do labirinto (8x4)' })).toBeVisible();
  await expect(page.getByText('Mapeando', { exact: true })).toBeVisible();
  await expect(page.getByText('11.20 V', { exact: true })).toBeVisible();
  await expect(page.getByText('1.20 m/s', { exact: true })).toBeVisible();
  await expect(page.getByText('1.18 m/s', { exact: true })).toBeVisible();
  await expect(page.getByText(/Erro do controle PID\s*0\.040/)).toBeVisible();
  await expect(page.getByText('Robô na célula (3, 1) · virado para N')).toBeVisible();
});
