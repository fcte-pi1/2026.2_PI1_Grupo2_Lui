import { defineConfig, devices } from '@playwright/test';

/**
 * A suíte E2E padrão usa respostas HTTP e WebSocket determinísticas para testar
 * o frontend sem depender de um robô físico ou de dados locais do PostgreSQL.
 * O caso e2e/integracao.spec.ts é opt-in e usa o backend real quando habilitado.
 */
export default defineConfig({
  testDir: './e2e',
  testMatch: '**/*.spec.ts',
  fullyParallel: true,
  forbidOnly: Boolean(process.env.CI),
  retries: process.env.CI ? 2 : 0,
  workers: process.env.CI ? 2 : undefined,
  reporter: process.env.CI ? [['list'], ['html', { open: 'never' }]] : 'list',
  timeout: 30_000,
  expect: { timeout: 5_000 },
  use: {
    baseURL: 'http://127.0.0.1:5173',
    trace: 'retain-on-failure',
    screenshot: 'only-on-failure',
    video: 'retain-on-failure',
    ...devices['Desktop Chrome'],
  },
  projects: [
    {
      name: 'chromium',
      use: { ...devices['Desktop Chrome'] },
    },
  ],
  webServer: {
    command: 'npm run dev -- --host 127.0.0.1',
    url: 'http://127.0.0.1:5173',
    reuseExistingServer: !process.env.CI,
    timeout: 30_000,
    env: {
      VITE_API_URL: 'http://localhost:3000',
      VITE_WS_URL: 'ws://localhost:3000/ws?role=dashboard',
    },
  },
});
