# Rodar o Micromouse sozinho — VS Code + PlatformIO no Mac

Este guia descreve a **nova versão preparada em 17/11/2026**. Ela foi compilada, mas **ainda não foi gravada na ESP32**. O comando `g` só terá espera de 5 segundos depois de você gravar essa versão. A versão anterior na placa começa a tentativa imediatamente após a preparação.

Você pode compilar e estudar o código agora. A gravação para teste físico e a movimentação ficam para depois de confirmar alimentação dos HC-SR04 e adaptação dos sinais ECHO. O software não mede a tensão desses fios.

## 1. Abrir o projeto correto

No VS Code, use **File → Open Folder…** e abra:

```text
/Users/gustavo/UnB/2026.2_PI1_Grupo2_Lui/src/firmware
```

É a pasta que contém `platformio.ini`. Não crie outro projeto e não abra uma pasta de `firmwareTest` para executar a missão completa. Preserve as modificações locais da branch `Softaware/Arduino`.

Se a extensão ainda não aparecer, instale **PlatformIO IDE**, publicada pelo PlatformIO, e aguarde a inicialização. Ela inclui o Core; não é necessário instalar outra cópia com `pip`. O projeto já fixa a plataforma e o framework compatíveis. [Documentação oficial](https://docs.platformio.org/en/latest/integration/ide/vscode.html)

## 2. Entender as três ações

| Ação | O que faz | Precisa da ESP? |
|---|---|---|
| Build | Transforma o código em firmware e procura erros de compilação | Não |
| Upload | Grava esse firmware na ESP e reinicia a placa | Sim |
| Monitor | Mostra mensagens e envia comandos pela USB | Sim |

A bateria não é necessária para compilar nem para gravar pela USB. Para movimentar o robô, o subsistema dos motores precisa estar alimentado.

## 3. Compilar agora, sem movimentar

Na lateral do PlatformIO, abra **PROJECT TASKS → esp32dev → General → Build**. Aguarde `SUCCESS`.

Alternativa pelo terminal integrado do PlatformIO:

```bash
cd /Users/gustavo/UnB/2026.2_PI1_Grupo2_Lui/src/firmware
pio run -e esp32dev
```

Se um terminal comum disser `pio: command not found`, use o executável que já existe neste Mac:

```bash
/Users/gustavo/.platformio/penv/bin/pio run -e esp32dev
```

Compilar não altera a placa e não faz o robô andar. Sucesso de compilação não comprova que sensores e montagem estão corretos.

## 4. Configurar o percurso antes de gravar

A configuração padrão é **4 colunas × 4 linhas**, partida no canto superior esquerdo, frente para Leste, objetivo no canto inferior direito. Os nomes descrevem coordenadas do mapa; alinhe fisicamente a frente do robô a essa convenção.

```text
        Norte
      (0,0) → Leste
       início ───────┐
          │          │
          │          │
          └────── objetivo (3,3)
                Sul
```

Em `src/config/network_config.local.hpp`, se esse arquivo existir, as opções abaixo são utilizadas pela nova configuração. Se não existir, os padrões já são 4/0/1. Há um exemplo em `network_config.local.example.hpp`.

```cpp
#define MM_MAZE_WIDTH 4       // 4, 8 ou 12; altura sempre 4
#define MM_START_CORNER 0     // 0=superior esquerdo; 1=superior direito;
                              // 2=inferior esquerdo; 3=inferior direito
#define MM_INITIAL_HEADING 1  // 0=Norte; 1=Leste; 2=Sul; 3=Oeste
```

O destino é o canto oposto ao início; não é o centro de um labirinto Micromouse clássico. Se a regra da competição mudar, o contrato de objetivo precisa ser alterado no programa e nos testes.

A célula tem **18 cm de passo**: 17 cm livres + parede de 1 cm. O ponto de referência do movimento é o eixo das rodas. Ter aproximadamente 3 cm entre um sensor e a parede não comprova sozinho que esse eixo esteja no centro. Primeiro valide um avanço de uma célula e uma curva de 90° na montagem real.

## 5. Gravar — depois da conferência elétrica

1. Deixe a bateria dos motores desligada e conecte a ESP ao USB de dados.
2. Feche Arduino IDE, outros monitores e consoles que estejam usando a mesma porta.
3. No PlatformIO, escolha **esp32dev → General → Upload**.
4. Aguarde `SUCCESS`. O nome da porta deve ser o da ESP conectada.

Alternativa pelo terminal:

```bash
pio device list
pio run -e esp32dev -t upload --upload-port /dev/cu.usbserial-0001
```

`/dev/cu.usbserial-0001` foi a porta usada nos nossos testes; confira a lista, pois o nome pode mudar. A posição física da porta USB no Mac não define o nome serial.

Se aparecer erro de porta ocupada, feche o programa que a está usando e tente novamente. Se ficar em `Connecting...`, segure BOOT até começar a gravação e solte. Não altere os pinos ou a alimentação dos sensores para corrigir um erro de upload.

## 6. Abrir o monitor e testar parado

Escolha **esp32dev → General → Monitor**, a 115200 baud. Ou:

```bash
pio device monitor -e esp32dev -p /dev/cu.usbserial-0001 -b 115200
```

Aguarde a calibração da IMU com o robô imóvel e a mensagem **PRONTO E PARADO**. Gravar, ligar a bateria ou pressionar EN não inicia a missão automaticamente.

Clique no terminal do monitor para dar foco. Envie **uma letra por comando**, em minúscula. O monitor padrão envia os caracteres conforme você digita; Enter não é necessário. Se você adicionou `send_on_enter` aos filtros, pressione Enter para enviá-los. Não digite frases: cada letra pode corresponder a um comando.

| Comando | Resultado |
|---|---|
| `h` | Mostra a ajuda |
| `s` | Mostra último quadro corrigido, idade, validade e motores ON/OFF |
| `d` | Mostra eco bruto e distância corrigida; confira a idade do quadro |
| `i` | Para motores, mede 10 vezes cada sensor e termina com sensores **pausados** |
| `p` | Para a tentativa e pausa os disparos dos sensores |
| `r` | Retoma somente os sensores; não inicia motores nem retoma missão |
| `x` | Interrompe/cancela a tentativa e desabilita motores |
| `g` | Prepara missão autônoma e arma partida após **5 segundos** |
| `t` | Teste guiado específico: até duas células, aproximar, esquerda, uma célula |
| `c` | Continuação guiada da esquina: aproximar, esquerda, uma célula |

`s` e `d` não fazem uma medição nova se os sensores estiverem pausados. Para medir parado, use `i`, ou `r` e depois `s`. `t` e `c` são testes de movimento e não são comandos de diagnóstico parado.

**Ctrl+C fecha o monitor; não é comando de parada do robô. Envie `x` antes de fechar, se quiser interromper uma tentativa.** [Monitor oficial do PlatformIO](https://docs.platformio.org/en/latest/core/userguide/device/cmd_monitor.html)

## 7. Iniciar a missão completa — após validar a montagem

1. Reinicie com EN; aguarde o fim da calibração, com o robô imóvel.
2. Posicione o eixo no início e a frente na direção configurada. Use as verificações estáticas de sensores; leitura inválida ou divergente pede investigação.
3. Ligue a bateria quando a montagem estiver pronta para movimentar e o caminho estiver desimpedido.
4. Se usou `i` ou `p`, envie `r` para retomar o sampler.
5. Envie `g`. Durante a preparação da IMU e a espera de 5 segundos, os motores ficam desligados. Espere a mensagem `MISSAO ARMADA` para contar os 5 segundos; `x` cancela.
6. Ao aparecer `TENTATIVA INICIADA`, o robô escolhe a rota a partir das paredes medidas. O limite é **180 segundos**, contado após a espera. O algoritmo não precisa receber de você a sequência de curvas.
7. No objetivo, deve aparecer `Objetivo alcançado` e os motores são desabilitados. Em falha, leia a mensagem e reposicione antes de outra tentativa.

Há uma tentativa por reinicialização; reiniciar também reinicializa a missão/mapa ativo. O mapa é salvo para diagnóstico, mas a tentativa seguinte não retoma automaticamente a anterior. O contador de três tentativas da configuração não representa três partidas automáticas.

O USB pode prender o robô. Para executar sem cabo, primeiro confirme que a bateria mantém a ESP alimentada ao retirar o USB. Só então utilize a espera de 5 segundos para retirar o cabo mantendo o robô imóvel. Se a placa reiniciar, ela voltará a aguardar um comando; não retomará sozinha. Sem USB, `x` não está disponível pelo monitor: a chave da bateria dos motores precisa estar acessível.

## 8. Testes no computador

Na pasta do firmware:

```bash
pio test -e motion_native
pio test -e native
```

O primeiro testa controlador, filtros e partida. O segundo testa mapa, percepção e navegação, incluindo 768 percursos gerados. As simulações usam medições/movimentos ideais; não comprovam precisão física, espaço de giro ou conclusão de qualquer labirinto em 3 minutos.

O benchmark de `test_position_tracker` foi corrigido nesta versão: verifica o orçamento absoluto de 500 µs por operação, preservado do teste anterior. Foi removida a comparação percentual entre trechos de trabalho diferente, que oscilava no computador. Esse teste no Mac não mede o prazo real do controle na ESP32. Em qualquer falha, leia o teste e o log antes de continuar.

## Como o código se organiza

- `main.cpp`: inicialização, comandos e coordenação da tentativa.
- `config/run_config.hpp`: formato, canto, orientação e espera de partida.
- `hal/SnapshotWallSensor.h`: converte um quadro real em observação válida ou desconhecida, sem disparos extras.
- `navigation/MazeMissionController`: observa, decide, gira, move e confirma posição. Retaguarda desconhecida exige giro de observação antes de mover.
- `navigation/MazeSolver`: Flood Fill/BFS; recalcula quando descobre paredes.
- `control/motion_controller`: executa deslocamentos com encoders e IMU; recebe correção lateral filtrada e limitada.
- `control/corridor_feedback_filter.hpp`: mediana de três quadros novos; não corrige erro sistemático de medição.

Os arquivos em `include/` são compatibilidade para outros projetos do repositório. Edite as implementações em `src/`.

## Correção de validade das leituras — 17/11/2026

A percepção agora invalida um quadro que venceu mesmo quando o timestamp não mudou, sem contar consultas repetidas como novas falhas de aquisição. Uma falha também limpa a média daquela direção, evitando misturar o primeiro valor válido recuperado com distâncias antigas. Os testes reproduziram os dois problemas antes da correção; após a alteração passaram 166 casos nas suítes native e motion_native, além da compilação esp32dev.

Esta revisão está no código de navegação e não foi gravada na placa. A ESP continua com o teste Bluetooth de yaw. A correção não compensa alimentação inadequada dos HC-SR04 nem substitui a validação física das distâncias.

## Partida pelo site — implementação preparada

O firmware final agora inicia o rádio já durante a preparação, anuncia seu estado e recebe comandos JSON do backend local. A função de preparação é compartilhada com `g`. Em uma partida remota válida, o tamanho, o canto e a orientação vêm do site; uma nova missão limpa mapa e filtros, recalibra imóvel e mantém a espera de 5 s. O mapa e a posição estimada são publicados após observações/movimentos e o percurso é encerrado ao chegar, parar ou falhar. É possível iniciar outra missão sem reiniciar a ESP, depois que o envio anterior terminar e o robô for reposicionado.

`MM_REMOTE_MOTION_ENABLED` permanece 0 por padrão: não liberar antes da validação da alimentação/ECHO dos HC-SR04 e dos movimentos físicos. A ESP atual continua com o diagnóstico Bluetooth atualizado, sem locomover; não foi gravado este firmware de navegação. Após a validação física, a liberação é feita no arquivo local de configuração e exige uma gravação do firmware final, mas nenhuma regravação entre tamanhos de labirinto.

Na missão iniciada pelo site, desconectar Bluetooth para a tentativa. Não existe retomada automática. O comando stop desabilita motores e é aceito independentemente da configuração de partida; ele ainda depende do enlace. O botão físico de bateria continua necessário para intervenção sem conexão.

Teste do parser compartilhado no Mac, na pasta do firmware:

```sh
clang++ -std=c++17 -I .pio/libdeps/esp32dev/ArduinoJson/src tests/remote_command.cpp -o /tmp/test_remote_command
/tmp/test_remote_command
```

## Teste real de partida pelo site — 17/11/2026

Após autorização explícita para testar com a alimentação atual dos sensores, foi adicionado o ambiente `esp32dev_remote_test`, que herda a navegação normal e define `MM_REMOTE_MOTION_ENABLED=1` e o nome Bluetooth já pareado `MicroMouse_BT_Test`. Este perfil **não simula nem bloqueia fisicamente os motores**: a condição de bancada confirmada foi rodas suspensas e bateria dos motores desligada. Não foram removidas as verificações de validade/idade dos sensores.

Esse perfil foi compilado e gravado na ESP. No site real, Iniciar missão recebeu ACK aceito (4×4), com espera de 5 s. Parar robô recebeu ACK e o estado passou a stopped cerca de 0,9 s após a confirmação da partida, antes de iniciar locomoção. A confirmação valida o comando e o cancelamento; não valida a precisão dos sensores nem uma travessia autônoma. A ESP atual passou a executar esse firmware de navegação de teste, substituindo BluetoothTelemetry. O ambiente padrão esp32dev continua com partida remota desabilitada por padrão.

## Diagnóstico de navegação real e prazo do sonar

O prazo do quadro passou de 150 para 270 ms, calculado a partir do ciclo sequencial de três sensores (cada um com timeout de 20 ms e intervalo quieto de 25 ms). O timestamp representa o início da aquisição mais antiga, e sua idade pode aproximar-se de 245 ms antes de publicar o quadro seguinte. Foram observados picos de 197 ms parado. Leituras inválidas continuam desconhecidas; o prazo ajustado não torna um eco ausente válido. Testes de limite/recuperação e navegação: 168 aprovados.

`robot_status.error_reason` agora inclui a leitura exata que disparou a parada frontal: ausência de eco, pulso, distância, tooClose ou reserva frontal válida. No teste real inferior direito/Norte, o robô confirmou uma célula de avanço e parou no trecho seguinte com pulso frontal=0us, distância=nan, tooClose=0. Esse evento não foi cancelamento do agente nem expiração do limite antigo. A última telemetria pós-parada voltou a medir 63,2 mm, mostrando por que é necessário preservar o evento em vez de inferir a causa pela leitura posterior. Não foi implementada recuperação automática de falhas de eco.

## Recuperação limitada de eco frontal — 17/11/2026

Durante avanço, uma leitura frontal inválida provoca frenagem imediata e pausa de recuperação, mantendo atualização de encoders/IMU. O alvo e a distância percorrida são preservados, inclusive eventual deslocamento durante a frenagem. O perfil de velocidade e os integradores recomeçam ao retomar. Exigem-se dois quadros novos e válidos consecutivos, pausa mínima de 200 ms e confirmação de assentamento das rodas/giroscópio. O mesmo quadro consultado novamente não conta como nova leitura.

O prazo é de 1200 ms por episódio e até três episódios por trecho. Falha persistente, quadro vencido, obstáculo dentro da reserva frontal ou tooClose encerram a tentativa. Parar pelo site permanece disponível e não há retomada após cancelamento. Encoder/IMU com falha também interrompem durante a pausa. O prazo do movimento exclui a espera para evitar falso stall/timeout; o limite total da missão de 180 s continua contando.

O site mostra "Freado · aguardando eco frontal". O backend informa recovering_front_echo e front_echo_recoveries. Passaram 177 testes de navegação/controle, 99 do backend e 15 de navegador. Testes de Bluetooth foram isolados do estado físico da API do robô.

A revisão foi gravada em esp32dev_remote_test. No ensaio real subsequente não houve pausa de recuperação (contador zero): a leitura frontal válida de 3,77 cm ficou abaixo da reserva de 5 cm e o robô parou. A retomada por perda de eco foi validada em testes no computador, mas não exercitada nesse ensaio físico. A aproximação da esquina permanece como ajuste separado.

## Aproximação autônoma da parede antes da curva

O avanço normal habilita o modo lento quando uma leitura frontal válida chega a 12 cm. O limite de velocidade passa a 20 mm/s e permanece até terminar o trecho. O modo desabilita a correção lateral (aberturas não puxam o robô para o lado), mantendo o controle de rumo pela IMU. O prazo desse trecho aumenta para até 12 s, sem alterar os 180 s da missão.

Enquanto a velocidade medida ainda exceder 30 mm/s, a reserva frontal permanece 5 cm. Após desacelerar, a reserva passa a 2 cm e a referência de parada é 3 cm, com tolerância de 0,5 cm. Nos centímetros finais, o alvo pode ser refinado dentro de ±20 mm da célula nominal; não há extensão livre do avanço. Parede detectada antes dessa faixa provoca parada de diagnóstico.

Quando a parede foi usada para refinar o alvo ou frear a aproximação, confirmar o deslocamento exige quadro fresco, leitura frontal de 3±0,5 cm e avanço dos encoders dentro de ±20 mm do passo da célula. Células de corredor aberto são confirmadas pelo avanço dos encoders, sem exigir parede a 3 cm; em seguida aguardam observações válidas antes de outra ação. Se as referências usadas não forem confirmadas, a missão para com diagnóstico da chegada. Essas referências não substituem a medição física da posição do eixo nem comprovam a envoltória do chassi durante uma curva.

A recuperação de eco permanece ativa com o robô freado; ecos inválidos não viram corredor livre. O modo de aproximação é limpo antes da próxima operação. Testes de política, velocidade, refinamento limitado, prazo e transição para giro: suíte completa com 182 casos aprovados. Perfil compilado: esp32dev_remote_test.

A observação de cada célula agora aguarda até 1500 ms, com motores parados, por três quadros distintos e válidos após a chegada. Contagem usa o timestamp da percepção efetivamente processada, para não contar duas vezes um mesmo snapshot. Um quadro inválido reinicia a sequência. Se as leituras não estabilizarem, a tentativa termina com indicação de frente/esquerda/direita. Isso evita decidir a rota no exato instante de uma falha isolada.

Depois de um giro, a missão também retorna à observação antes de ordenar avanço; não usa medidas tomadas com os sensores voltados para a direção anterior. Os testes de missão e de janela de observação foram atualizados; suíte completa: 186 casos aprovados.

O ensaio revelou e corrigiu a aplicação excessiva da referência frontal a células comuns de corredor aberto. A confirmação agora distingue deslocamento por odometria de parada referenciada pela parede. Regressão final: 187 testes aprovados, incluindo célula confirmada a 176,9 mm com parede ainda a 21,1 cm.

No ensaio com a revisão final, a primeira partida foi recusada por leitura inválida; uma nova partida, sem movimentação entre elas, foi aceita. O robô confirmou uma célula e iniciou a seguinte. A recuperação de eco atuou uma vez (pausa e retomada registradas). Depois ocorreu uma leitura frontal de 4,42 cm enquanto as velocidades medidas ainda estavam acima do critério de desaceleração; manteve-se a reserva de 5 cm e a tentativa parou. A telemetria imediatamente anterior mostrava cerca de 20,7 cm, e a posterior 18,9 cm: a leitura curta foi inconsistente com o deslocamento observado. A aproximação/curva não foi validada fisicamente neste ensaio. O firmware permanece gravado, com o robô parado por proteção. Não foi removida a proteção para permitir avanço com leitura curta não confirmada.

## Filtro robusto dos três ultrassons

O sampler mantém uma janela de três tentativas por sensor. Com três ecos válidos, usa a mediana; com dois válidos e uma falha, usa a média dos dois valores válidos. Exige pelo menos dois ecos reais recentes. Uma perda lateral isolada pode manter a estimativa; duas falhas na janela, silêncio prolongado ou evidência com idade acima de 270 ms invalidam a distância. O timestamp da última evidência real é preservado. Leituras de controle repetidas não alimentam novamente a janela.

A aquisição bruta, os pinos, offsets e intervalos permanecem iguais. O diagnóstico USB s/d continua mostrando os ecos brutos; o site e o mapa recebem as estimativas. A percepção não aplica uma segunda média sobre a mediana do hardware. A correção lateral conserva seu filtro existente e desabilita a correção quando os ecos brutos laterais falham; o rumo continua pela IMU.

No frontal, a primeira leitura curta suspeita freia para conferir novas amostras. A missão só é encerrada por proximidade após duas leituras curtas em três quadros distintos. Uma leitura isolada de 1 cm entre leituras de 12 cm não vira encerramento imediato; dados ausentes continuam exigindo pausa e recuperação limitada. Obstáculo próximo confirmado, quadro vencido e perda persistente continuam interrompendo. A recuperação retoma apenas com dados válidos e freio assentado.

Foram aprovados 197 testes, incluindo 12–1–12 cm, salto válido isolado, mudança real de distância, perda lateral isolada/persistente, expiração, wrap do relógio, ausência de dupla filtragem e pausa/retomada com alvo preservado. Isso melhora o tratamento das leituras; não comprova que falhas elétricas ou acústicas do sensor foram corrigidas.

No teste físico do filtro, a telemetria registrou um giro para Oeste e avanço de uma célula estimada, além de uma pausa/retomada por conferência do frontal. A parada final foi código 7 (LeftWheelStalled): faltou progresso no encoder esquerdo, sem diagnóstico conclusivo da causa. Últimas distâncias filtradas: 115,61 mm à frente, 28,10 mm à esquerda e 29,60 mm à direita. O filtro não encerrou a missão pelo salto frontal isolado; a execução completa do labirinto continua sem validação. Robô ficou parado.
