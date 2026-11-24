# Site com Bluetooth real — como usar

A integração está rodando localmente no Mac:

**http://127.0.0.1:5173/?fonte=bluetooth**

Fluxo: ESP32 → Bluetooth Classic SPP → receptor nativo do Mac → backend → WebSocket → site.

O navegador recebe dados do backend; ele não precisa abrir o rádio Bluetooth diretamente. Mantenha a ESP no firmware **BluetoothTelemetry**, alimentada somente pelo USB enquanto a alimentação dos ultrassons não for corrigida. Esse firmware não movimenta o robô nem dispara os ultrassons.

## O que você verá

- Yaw relativo, giro no eixo Z e aceleração X/Y/Z reais da IMU.
- Memória livre, tempo ligado e sequência do pacote da ESP.
- Estado da conexão e tempo desde a última leitura.
- “Sem dados recentes” se parar de chegar telemetria por cerca de 2,5 segundos; o painel conserva os últimos valores, sem apresentá-los como novos.
- “Sem leitura” quando a IMU declarar medida inválida, em vez de inventar zero.

Distância das paredes, bateria, velocidade das rodas e mapa não são medidos neste teste. **Nenhuma sessão de corrida é criada ou persistida nesse modo.** O yaw é relativo; a versão atual do teste calibra por mais tempo e compensa o repouso. Isso não fornece norte absoluto, e giros abaixo de 0,25°/s podem ser confundidos com repouso.

## Para iniciar sozinho depois

Abra dois terminais. No primeiro:

```bash
cd /Users/gustavo/UnB/2026.2_PI1_Grupo2_Lui/src/backend
npm run dev:bluetooth
```

No segundo:

```bash
cd /Users/gustavo/UnB/2026.2_PI1_Grupo2_Lui/src/frontend
npm run dev:bluetooth
```

Abra **http://127.0.0.1:5173/?fonte=bluetooth**. Espere o indicador “Recebendo dados reais” e veja o contador de pacotes avançar. O link com `?fonte=bluetooth` abre o monitor parado; a página normal e a simulação do projeto continuam disponíveis separadamente.

Os dois serviços foram iniciados durante esta tarefa. Você não precisa iniciá-los de novo se o link já abrir. Mantenha os terminais ativos quando estiver operando por conta própria. Para encerrar, pressione Ctrl+C em cada terminal. Encerrar o backend fecha seu receptor Bluetooth. Fechar apenas a aba do site não encerra o backend nem movimenta o robô.

## Preparação em outro computador ou após baixar o projeto

1. Instale Node.js compatível com as dependências do projeto e as ferramentas C/C++ do macOS. Neste Mac, as dependências já foram instaladas.
2. Execute `npm ci` em `src/backend` e em `src/frontend`.
3. Na pasta do backend, copie `.env.bluetooth.example` para `.env.bluetooth.local` e coloque o endereço Bluetooth da sua ESP. Esse arquivo local fica ignorado pelo Git. A configuração atual deste Mac já está preenchida.
4. Ainda em `src/backend`, execute `npm run bluetooth:build` para compilar o receptor com o framework IOBluetooth do macOS.
5. Pareie `MicroMouse_BT_Test` nos Ajustes do Sistema, se ainda não estiver pareado, e execute os dois comandos de inicialização acima.

O monitor usa `MM_DIAGNOSTIC_ONLY=1` e dispensa PostgreSQL. O fluxo normal do backend, com persistência de corridas, permanece separado e exige configuração de `DATABASE_URL`, geração do Prisma e banco disponível. Não foi configurado um banco nem executada uma migração nesta etapa.

## Se não aparecerem dados

- Feche qualquer receptor Bluetooth separado que esteja usando a ESP. O backend deve ser o cliente SPP desse teste.
- Confira a alimentação USB da ESP e o firmware gravado. O nome anunciado é `MicroMouse_BT_Test`.
- Veja o terminal do backend. Ele tenta reconectar automaticamente se o receptor cair.
- A porta virtual `/dev/cu.MicroMouse_BT_Test` falhou após regravação neste Mac; o monitor usa acesso nativo RFCOMM, que funcionou nos testes.
- Se o macOS solicitar acesso a Bluetooth para o aplicativo/Terminal, permita para este teste.
- Se a porta 3000 ou 5173 estiver ocupada, confira se outra instância do backend/site já está aberta; evite iniciar cópias concorrentes.
- A seleção “Simulação” mostra dados simulados do site. Use “Bluetooth real” para conferir a ESP.

No firmware de teste, `p`/`x` no monitor USB pausam o envio e `r` retoma. Eles não acionam motores. Isso permite conferir o aviso de ausência de dados; não é necessário usar esses comandos para a transmissão normal.

## Verificação realizada

- 92 testes do backend aprovados.
- 11 testes do navegador aprovados; 1 teste de persistência real com PostgreSQL ficou desativado, como previsto na suíte.
- Backend, receptor nativo e site compilaram.
- Lint dos arquivos frontend alterados aprovado. O lint global ainda acusa problemas anteriores em MazeMap, SessionOverview e Historico; foram registrados na revisão, sem desativar regras para escondê-los.
- Verificação com a ESP real: o contador na página avançou de 544 para 550, a página foi recarregada e continuou recebendo o quadro atual. Nenhum erro JavaScript de página nessa verificação.
- Conferido em desktop e largura de celular, sem rolagem horizontal.

## Arquivos principais

- `src/backend/src/communication/bluetoothNative.ts`: mantém o receptor nativo e reconecta.
- `src/backend/src/communication/bluetoothDiagnostic.ts`: valida e normaliza o pacote real.
- `src/backend/src/ws/telemetry.gateway.ts`: distribui aos dashboards e preserva idade no replay.
- `src/frontend/src/components/BluetoothPanel.tsx`: painel de diagnóstico parado.
- `src/firmwareTest/BluetoothTelemetry/tools/receber_mac.m`: receptor nativo compartilhado com o teste de rádio.

## Controle da missão pelo site

A página principal `http://127.0.0.1:5173/` conserva o mapa e inclui **Missão no labirinto**, com tamanho 4×4/8×4/12×4, canto inicial, orientação, **Iniciar missão** e **Parar robô**. Selecionar outra configuração ou reconectar o painel não inicia movimentos e não altera o mapa em andamento. Uma nova mensagem `maze` da ESP limpa o mapa para a próxima missão. A meta é o canto diagonalmente oposto; a posição apresentada é uma estimativa dos sensores/encoders.

Os comandos percorrem site → HTTP local → backend → stdin do receptor nativo → Bluetooth SPP → ESP. O sucesso só aparece após `robot_ack` com o ID correspondente. Não há fila offline nem repetição automática de partida; se a confirmação de início não chegar em 10 s, o backend solicita parada e informa a falha. O botão de parada depende do enlace e não substitui a chave física.

### Estado atual e liberação futura

A ESP está com **BluetoothTelemetry** atualizado: confirma `stop`, mas nunca executa `start`; motores e disparos ultrassônicos continuam desativados. A correção de yaw permanece. O site informa essa limitação e desabilita Iniciar.

O firmware final em `src/firmware` foi preparado e compilado para receber os comandos, mudar de formato entre missões, publicar mapa/posição e exportar o percurso ao término. Ainda não foi gravado nem validado fisicamente neste fluxo. `MM_REMOTE_MOTION_ENABLED` é **0 por padrão**. Depois da correção e validação elétrica dos HC-SR04 e do teste físico dos movimentos, defina `#define MM_REMOTE_MOTION_ENABLED 1` em `src/firmware/src/config/network_config.local.hpp` e grave o firmware final uma vez. A troca 4×4 → 8×4 → 12×4 passa então a ocorrer pelo site, sem regravação por tamanho.

O início compartilha a preparação do comando USB `g`: calibração imóvel, espera de 5 s e limite de 180 s após a espera. O firmware aceita outra missão somente parado e depois de terminar o envio anterior. Limpa mapa, filtros e referências de movimento. Durante uma missão iniciada remotamente, perder o enlace Bluetooth interrompe a tentativa; reconectar não a retoma. Um novo nonce de conexão invalida partidas antigas.

### API e protocolo

- `GET /api/robot/status`: último estado da ESP e `connected` calculado pela idade (<2,5 s).
- `POST /api/robot/command`: `{ "action":"start", "width":4, "corner":0, "heading":1 }` ou `{ "action":"stop" }`. Disponível apenas por conexão local; origem de site externo é rejeitada.
- ESP anuncia `robot_status`: firmware `maze`/`diagnostic`, `boot_id` (nonce renovado em cada conexão), `remote_enabled`, `state`, `width`, `corner`, `heading`.
- Backend envia `robot_command` com ID hexadecimal de 32 caracteres e nonce atual; ESP responde `robot_ack` com `id`, `accepted`, `message`.
- Canto: 0 superior esquerdo, 1 superior direito, 2 inferior esquerdo, 3 inferior direito. Orientação: 0 Norte, 1 Leste, 2 Sul, 3 Oeste. Linhas do firmware crescem para Sul; o adaptador existente inverte y para o desenho do painel.

O modo monitor (`MM_DIAGNOSTIC_ONLY=1`) agora permite ver mapas e telemetria de navegação ao vivo, mas continua sem persistência/histórico. Para histórico, use o modo normal com PostgreSQL configurado. Não foi criado um banco nesta etapa.

### Atualização após o teste real de Iniciar

Por solicitação explícita do usuário, com rodas suspensas e bateria dos motores desligada, foi gravado `esp32dev_remote_test` da navegação. Iniciar missão foi aceito pela ESP e cancelado com Parar robô cerca de 0,9 s após o ACK de início, antes do fim da espera de 5 s. Agora a ESP anuncia firmware maze e permite partida pelo site; o nome Bluetooth pareado permanece MicroMouse_BT_Test. Não houve ensaio de locomoção nesse teste. As observações anteriores sobre a ESP estar no diagnóstico descrevem a etapa anterior a essa regravação. O perfil de teste habilita a navegação real, não um simulador: manter a condição de bancada enquanto o teste depender da montagem atual.

### Recuperação do eco frontal

A navegação atual pode frear temporariamente para recuperar o ultrassom frontal. robot_status.recovering_front_echo indica a pausa e front_echo_recoveries conta episódios na missão. O site mostra "Freado · aguardando eco frontal", mantém Parar disponível e bloqueia outra partida enquanto a missão está em andamento. A retomada exige duas leituras novas e válidas e rodas assentadas. Há limites de 1200 ms por pausa e três episódios por trecho; o limite total da missão permanece em 180 s.
