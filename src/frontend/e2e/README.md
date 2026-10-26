# Testes funcionais E2E — Playwright

Os testes simulam interações reais na interface do MicroMouse, mas os cenários padrão não exigem ESP32, backend ou PostgreSQL: as APIs HTTP e as mensagens WebSocket são controladas pelo próprio teste. Isso mantém a suíte repetível e independente dos dados existentes na máquina.

## Instalação e execução

Na pasta `src/frontend`:

```bash
npm install
npx playwright install chromium
npm run test:e2e
```

Para abrir o executor interativo:

```bash
npm run test:e2e:ui
```

Para ver o relatório HTML da última execução:

```bash
npm run test:e2e:report
```

A configuração inicia o Vite em `http://127.0.0.1:5173`. Feche um servidor Vite preexistente se precisar assegurar que a variável de ambiente definida pela configuração seja aplicada. Para executar os testes padrão, não é necessário iniciar o backend nem o PostgreSQL.

## Cobertura incluída

- `dashboard.spec.ts`: renderização do painel, seleção dos labirintos 4x4/8x4/12x4, reinício e erro da API.
- `simulacao.spec.ts`: corrida simulada sem hardware e métricas de telemetria.
- `historico.spec.ts`: consulta de sessão, trajeto, filtro por severidade e navegação de volta.
- `telemetria.spec.ts`: envia mensagens determinísticas via WebSocket interceptado e verifica os indicadores atualizados na interface.
- `integracao.spec.ts`: teste opt-in do fluxo real dispositivo-WebSocket → gateway → PostgreSQL → API → interface. Ele cria uma sessão e um evento no banco e persiste pontos do trajeto; por isso, deve ser executado apenas contra um banco de teste exclusivo.

## Teste de integração real com backend e PostgreSQL

O teste real é ignorado por padrão. Para executá-lo:

1. Configure `DATABASE_URL` no ambiente do backend para apontar para um banco de teste, não para a base compartilhada de desenvolvimento.
2. Na pasta `src/backend`, gere o Prisma Client e aplique as migrações de acordo com a configuração deste repositório:

   ```bash
   npx prisma generate
   npx prisma migrate deploy
   npm run dev
   ```

3. Com o backend acessível em `http://localhost:3000`, abra outro terminal, vá para `src/frontend` e execute:

   ```bash
   npm run test:e2e:backend
   ```

O teste transmite pacotes de `session_start`, `telemetry`, `log` e `session_end` pelo endpoint WebSocket `ws://localhost:3000/ws?role=device`. Em seguida, verifica o estado concluído da sessão, o evento registrado e os pontos persistidos consultando a API real e a tela de histórico.

Cada execução deixa registros de teste no banco configurado. Use um banco dedicado/descartável para E2E; não configure a suíte contra um banco de produção ou compartilhado.

## Relatórios e diagnóstico

- Resultado resumido: saída do terminal.
- Falhas: screenshot, vídeo e trace retidos conforme a configuração Playwright.
- Relatório HTML: criado em `playwright-report/` quando solicitado pela configuração/CLI.
