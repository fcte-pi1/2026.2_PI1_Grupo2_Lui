# Frontend: painel da corrida (HU-08 e HU-09)

```bash
npm install
npm run dev
```

- `http://localhost:5173/` escuta o backend em `ws://localhost:3000/ws`.
- `http://localhost:5173/?fonte=simulador` roda uma corrida simulada (labirinto aleatório), pra testar sem o robô.
  Também dá pra trocar no seletor do topo.

## Onde está cada coisa

| Arquivo | O que faz |
|---|---|
| `src/pages/Dashboard.tsx` | Tela principal, escolhe a fonte (robô ou simulação) e o tamanho do labirinto |
| `src/components/MazeMap.tsx` | HU-08: mapa em SVG (grade, paredes, células visitadas, robô com seta de orientação) |
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

Hoje o backend (`telemetry.gateway.ts`) só recebe e loga; repassar essas mensagens ao dashboard faz parte da HU-10.
