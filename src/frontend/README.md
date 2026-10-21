# Frontend: painel da corrida e histórico (HU-08, HU-09, HU-11 e HU-12)

```bash
npm install
npm run dev
```

- `http://localhost:5173/` escuta o gateway em `ws://localhost:3000/ws?role=dashboard` (sobrescreva com `VITE_WS_URL` se necessário).
- `http://localhost:5173/?fonte=simulador` roda uma corrida simulada (labirinto aleatório), pra testar sem o robô.
  Também dá pra trocar no seletor do topo.

## Onde está cada coisa

| Arquivo | O que faz |
|---|---|
| `src/App.tsx` | Alterna entre painel ao vivo e histórico, compartilhando a lista de sessões entre as telas |
| `src/pages/Dashboard.tsx` | Tela principal, escolhe a fonte (robô ou simulação) e o tamanho do labirinto |
| `src/components/SessionOverview.tsx` | HU-11/HU-12: resumo de execuções registradas e últimos eventos dentro do painel ao vivo |
| `src/pages/Historico.tsx` | HU-11/HU-12: consulta sessões, trajetos e eventos com filtro por severidade |
| `src/services/api.ts` | Cliente HTTP único para sessões, trajetos e logs, com tipos alinhados às respostas do backend |
| `src/utils/formatters.ts` | Formatadores compartilhados de data, tempo, duração e estado de conclusão |
| `src/components/MazeMap.tsx` | HU-08: mapa em SVG (grade, paredes, células visitadas, trilha percorrida e robô com orientação) |
| `src/components/StatusPanel.tsx` | HU-09: bateria com alerta, velocidade das rodas, estado do robô |
| `src/components/PidChart.tsx` | HU-09: gráfico contínuo do erro PID (Chart.js) |
| `src/services/dashboardStore.ts` | Estado global; `aplicarMensagem` aplica cada mensagem recebida |
| `src/services/bateria.ts` | Limites de tensão (ajustar quando a bateria do robô for definida) |
| `src/services/simulador.ts` | Corrida simulada |
| `src/types/telemetry.ts` | Formato das mensagens |

## Mensagens que o painel espera pelo WebSocket

Todas em JSON, com o campo `type`. Coordenadas: `(0,0)` é a largada, no canto inferior esquerdo; `y` cresce para o Norte.

```json
{ "type": "maze", "width": 8, "height": 4 }
{ "type": "cell", "x": 3, "y": 1, "visited": true,
  "walls": { "north": true, "south": false, "east": true, "west": false } }
{ "type": "position", "x": 3, "y": 1, "heading": "N" }
{ "type": "telemetry", "battery_voltage": 11.2, "pid_error": 0.04,
  "wheels_speed": { "left": 1.2, "right": 1.18 },
  "robot_state": "MAPPING", "error_reason": "Colisão", "timestamp": 1700000000000 }
```

`heading`: `N`, `S`, `L` ou `O`. `robot_state`: `MAPPING`, `NAVIGATING`, `RETURNING`, `STOPPED`, `ERROR` ou `FINISHED`
(`error_reason` só aparece com `ERROR`). `timestamp` em milissegundos.

### Pacote do firmware (HU-00)

O backend também pode repassar, sem converter, o pacote que o firmware já gera em `MazeMap::cellPacketJson`:

```json
{ "robot": { "x": 3, "y": 1, "heading": "NORTH" },
  "cell":  { "x": 3, "y": 1, "visited": true,
             "walls": { "north": true, "south": null, "east": true, "west": false } } }
```

No firmware a origem `(0,0)` é a célula de largada e `y` cresce para o Sul. O adaptador
`src/services/adaptadorFirmware.ts` transforma esse pacote nas mensagens `cell` e `position`
acima, só invertendo o eixo `y` (`y_painel = altura - 1 - y`). Ele precisa da altura do
labirinto, por isso a mensagem `maze` deve ser enviada antes de cada corrida.

O gateway da HU-10 aceita conexões com `role=device` para mensagens do ESP32 e `role=dashboard` para navegadores. Ele persiste início/fim de sessão, logs e telemetria no backend e retransmite ao dashboard o mapa e o status mais recente; ao reconectar o navegador, envia um snapshot de mapa/telemetria. Lados de parede podem ser `true` (parede), `false` (passagem medida) ou `null` (desconhecido); o estado do mapa preserva informação anterior e espelha lados conhecidos entre células vizinhas.


## Integração do histórico e diagnósticos (HU-11/HU-12)

O cabeçalho do painel possui a ação **Histórico e diagnósticos**, que abre a consulta das sessões persistidas, pontos do trajeto e eventos filtráveis por severidade. O mapa ao vivo destaca a trilha derivada das mensagens de posição, sem duplicar a persistência do firmware. O painel ao vivo também mostra as quatro sessões mais recentes e solicita ao backend apenas os cinco eventos finais da sessão mais recente, sem carregar todo o log para produzir o resumo. O resumo pode ser atualizado manualmente e é solicitado novamente quando o backend confirma eventos de sessão (`session_started`, `event_recorded` ou `session_finished`), evitando consultas repetidas a cada atualização de telemetria.

As telas compartilham a mesma lista de sessões carregada por `App.tsx`; a navegação apenas oculta a tela inativa, mantendo a conexão WebSocket e o estado do mapa/telemetria ativos. Se o backend estiver indisponível, o painel de corrida continua utilizável e exibe um erro localizado no resumo do histórico.
