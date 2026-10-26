import { expect, test } from '@playwright/test';

/**
 * E2E real, opt-in: precisa de backend em localhost:3000, DATABASE_URL válido,
 * migrações aplicadas e Prisma Client gerado. Use um banco exclusivo de teste.
 */
test('persiste uma corrida recebida pelo WebSocket e recupera sessão, trajeto e evento pela interface', async ({ page, request }) => {
  test.skip(
    process.env.E2E_REAL_BACKEND !== '1',
    'Teste de integração real desabilitado por padrão; defina E2E_REAL_BACKEND=1 e prepare o backend/PostgreSQL.',
  );

  const health = await request.get('http://localhost:3000/health');
  expect(health.ok(), 'O backend deve estar acessível em http://localhost:3000/health').toBeTruthy();

  // Identificadores válidos para o contrato do backend. O ID da sessão do banco
  // será obtido pelo ACK session_started, evitando selecionar uma sessão antiga.
  const attempt = Math.floor(Math.random() * 65_535) + 1;
  const deviceSessionId = (Date.now() % 0xffff_ffff) + 1;
  const descricaoEvento = `Playwright E2E persistencia ${deviceSessionId}`;

  await page.goto('/');
  await expect(page.getByRole('heading', { name: 'MicroMouse · Painel da corrida' })).toBeVisible();

  // Simula a ESP32, mas aguarda os ACKs emitidos pelo gateway real. Se o backend
  // rejeitar algum pacote, o erro gateway_error devolvido ao dispositivo aparece
  // na mensagem de falha em vez de terminar apenas num timeout de status.
  const protocolo = await page.evaluate(async ({ deviceSessionId, attempt, descricaoEvento }) => {
    type Packet = Record<string, unknown>;
    const dashboardMessages: Packet[] = [];
    const deviceErrors: string[] = [];
    const waiters: Array<{
      matches: (packet: Packet) => boolean;
      resolve: (packet: Packet) => void;
      reject: (error: Error) => void;
      timer: number;
    }> = [];

    const dashboard = new WebSocket('ws://localhost:3000/ws?role=dashboard');
    const device = new WebSocket('ws://localhost:3000/ws?role=device');

    const parsePacket = (data: unknown): Packet | null => {
      try {
        const value: unknown = JSON.parse(String(data));
        return typeof value === 'object' && value !== null && !Array.isArray(value)
          ? value as Packet
          : null;
      } catch {
        return null;
      }
    };

    dashboard.addEventListener('message', (event) => {
      const packet = parsePacket(event.data);
      if (!packet) return;
      const index = waiters.findIndex((waiter) => waiter.matches(packet));
      if (index >= 0) {
        const [waiter] = waiters.splice(index, 1);
        window.clearTimeout(waiter.timer);
        waiter.resolve(packet);
      } else {
        dashboardMessages.push(packet);
      }
    });

    device.addEventListener('message', (event) => {
      const packet = parsePacket(event.data);
      if (packet?.type === 'gateway_error') {
        deviceErrors.push(String(packet.message ?? 'erro não especificado'));
      }
    });

    const waitForDashboard = (matches: (packet: Packet) => boolean, description: string): Promise<Packet> => {
      const queuedIndex = dashboardMessages.findIndex(matches);
      if (queuedIndex >= 0) return Promise.resolve(dashboardMessages.splice(queuedIndex, 1)[0]);

      return new Promise<Packet>((resolve, reject) => {
        const waiter = {
          matches,
          resolve,
          reject,
          timer: 0,
        };
        waiter.timer = window.setTimeout(() => {
          const index = waiters.indexOf(waiter);
          if (index >= 0) waiters.splice(index, 1);
          const detail = deviceErrors.length > 0
            ? `; o gateway rejeitou pacote(s): ${deviceErrors.join(' | ')}`
            : '; nenhum gateway_error foi recebido pelo dispositivo';
          reject(new Error(`Timeout aguardando ACK '${description}'${detail}`));
        }, 6_000);
        waiters.push(waiter);
      });
    };

    const open = (socket: WebSocket, name: string): Promise<void> => new Promise((resolve, reject) => {
      const timer = window.setTimeout(() => reject(new Error(`Timeout ao abrir WebSocket ${name}`)), 6_000);
      socket.addEventListener('open', () => {
        window.clearTimeout(timer);
        resolve();
      }, { once: true });
      socket.addEventListener('error', () => {
        window.clearTimeout(timer);
        reject(new Error(`Não foi possível abrir WebSocket ${name}`));
      }, { once: true });
    });

    try {
      await Promise.all([open(dashboard, 'dashboard'), open(device, 'device')]);

      device.send(JSON.stringify({
        type: 'session_start',
        device_session_id: deviceSessionId,
        tipo_labirinto: '4x4',
        tentativa: attempt,
      }));
      const startAck = await waitForDashboard(
        (packet) => packet.type === 'session_started' && packet.device_session_id === deviceSessionId,
        'session_started',
      );
      const idSessao = Number(startAck.id_sessao);
      if (!Number.isInteger(idSessao) || idSessao <= 0) {
        throw new Error(`ACK session_started retornou id_sessao inválido: ${String(startAck.id_sessao)}`);
      }

      device.send(JSON.stringify({
        type: 'telemetry',
        session_id: deviceSessionId,
        t_ms: 50,
        battery_voltage: 11.1,
        battery_current_a: 0.2,
        pid_error: 0.03,
        wheels_speed: { left: 0.4, right: 0.42 },
        heading_deg: 173.25,
        robot_state: 'MAPPING',
        sensor_distances_mm: { front: 90, left: 120, right: 115 },
      }));
      await waitForDashboard(
        (packet) => packet.type === 'telemetry' && packet.heading_deg === 173.25,
        'telemetry',
      );

      device.send(JSON.stringify({
        type: 'log',
        session_id: deviceSessionId,
        nivel: 'INFO',
        mensagem: descricaoEvento,
        t_ms: 100,
      }));
      await waitForDashboard(
        (packet) => packet.type === 'event_recorded'
          && packet.session_id === deviceSessionId
          && packet.descricao === descricaoEvento,
        'event_recorded',
      );

      device.send(JSON.stringify({
        type: 'session_end',
        session_id: deviceSessionId,
        tempo_total_ms: 1_250,
        motivo: 'CHEGADA',
        stride: 1,
        tipo_labirinto: '4x4',
        tentativa: attempt,
        desafio_cumprido: true,
        consumo_bateria_mah: 0.4,
        pontos: [
          { t: 0, x: 0, y: 0, heading: 'NORTH' },
          { t: 250, x: 0, y: 1, heading: 'NORTH' },
          { t: 750, x: 1, y: 1, heading: 'EAST' },
        ],
      }));
      const endAck = await waitForDashboard(
        (packet) => packet.type === 'session_finished' && packet.device_session_id === deviceSessionId,
        'session_finished',
      );
      return { idSessao, endAck };
    } finally {
      for (const waiter of waiters) {
        window.clearTimeout(waiter.timer);
        waiter.reject(new Error('WebSocket de teste encerrado'));
      }
      if (dashboard.readyState === WebSocket.OPEN || dashboard.readyState === WebSocket.CONNECTING) dashboard.close();
      if (device.readyState === WebSocket.OPEN || device.readyState === WebSocket.CONNECTING) device.close();
    }
  }, { deviceSessionId, attempt, descricaoEvento });

  const idSessao = protocolo.idSessao;
  expect(protocolo.endAck).toMatchObject({ status_conclusao: 'CONCLUIDO', pontos_registrados: 3 });

  await expect.poll(async () => {
    const resposta = await request.get('http://localhost:3000/api/sessions');
    if (!resposta.ok()) return null;
    const sessoes = await resposta.json() as Array<{
      id_sessao: number;
      status_conclusao: string;
      numero_tentativa: number | null;
      tipo_labirinto: string | null;
    }>;
    return sessoes.find((sessao) => sessao.id_sessao === idSessao) ?? null;
  }, { timeout: 15_000 }).toMatchObject({
    id_sessao: idSessao,
    status_conclusao: 'CONCLUIDO',
    numero_tentativa: attempt,
    tipo_labirinto: '4x4',
  });

  // Consulta os dados persistidos antes de validar a representação pela interface.
  const [respostaLogs, respostaPercurso] = await Promise.all([
    request.get(`http://localhost:3000/api/sessions/${idSessao}/logs`),
    request.get(`http://localhost:3000/api/sessions/${idSessao}/navigation`),
  ]);
  expect(respostaLogs.ok()).toBeTruthy();
  expect(respostaPercurso.ok()).toBeTruthy();
  expect(JSON.stringify(await respostaLogs.json())).toContain(descricaoEvento);
  expect((await respostaPercurso.json() as unknown[])).toHaveLength(3);

  // Atualiza explicitamente a lista do dashboard; ela não é atualizada automaticamente
  // quando uma sessão é criada diretamente pelo cliente de teste via WebSocket.
  await page.getByRole('button', { name: 'Atualizar histórico' }).click();
  const resumo = page.getByRole('heading', { name: 'Execuções e diagnósticos' }).locator('xpath=../../..');
  await expect(resumo.getByText(new RegExp(`tentativa ${attempt}`))).toBeVisible({ timeout: 10_000 });

  // Confere o fluxo de consulta histórico -> trajeto + evento.
  await page.getByRole('button', { name: 'Histórico e diagnósticos' }).click();
  const listaSessoes = page.getByRole('complementary', { name: 'Sessões' });
  await listaSessoes.getByRole('button', { name: new RegExp(`tentativa ${attempt}`) }).click();
  const detalhes = page.getByRole('region', { name: 'Trajeto e eventos da sessão' });
  await expect(detalhes.getByRole('heading', { name: `Trajeto · sessão #${idSessao}` })).toBeVisible();
  await expect(detalhes.getByText(descricaoEvento)).toBeVisible();
  await expect(detalhes.getByText('3 pontos')).toBeVisible();
});
