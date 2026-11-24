# Bluetooth da ESP32 — teste de telemetria parada

Teste separado do firmware de navegação, em `src/firmwareTest/BluetoothTelemetry`. Usa o rádio integrado da ESP32 original e **Bluetooth Classic SPP**. Não é BLE.

## O que transmite

A cada 250 ms enquanto há cliente conectado:

- sequência, tempo ligado e memória livre da ESP;
- medidas reais da IMU: aceleração X/Y/Z, giro Z e yaw relativo;
- indicação de motores e ultrassons desabilitados.

As medidas da IMU usam o driver do projeto. Se ela não responder, aparecem `valid:false` e valores `null`. Yaw é orientação relativa integrada, sujeita a deriva; não é uma bússola absoluta.

Não há leituras de ultrassom, corrente ou tensão de bateria neste teste. Nenhuma distância é inventada. Os TRIGs ficam LOW, o nSLEEP da ponte H fica LOW e as entradas dos motores ficam LOW. Isso não corta o VCC dos sensores. A alimentação/ECHO dos HC-SR04 continuam pendentes de conferência.

O tipo do pacote é `bt_telemetry_test`, JSON por linha. O backend atual usa outro contrato (`session_start`/`telemetry`); este teste valida o enlace e as medidas, sem iniciar servidor, banco de dados ou dashboard. O transporte de navegação existente em `src/firmware/src/communication/TelemetryClient` permanece preservado.

## Receber no Mac — método validado

O Mac já tem `MicroMouse_BT_Test` pareado. A primeira captura pela porta `/dev/cu.MicroMouse_BT_Test` funcionou, mas essa porta virtual deixou de entregar dados depois da regravação. Abrir a porta sem erro não confirmou conexão ativa. O receptor nativo abaixo abriu o canal RFCOMM/SPP diretamente e recebeu dados novamente, sem depender dessa porta virtual.

No Terminal:

```bash
cd /Users/gustavo/Documents/Codex/2026-11-12/estou-desenvolvendo-o-projeto-do-rob/outputs/bluetooth
./receber_mac FC:B4:67:8C:48:E2 minha-captura.jsonl 60
```

O endereço é o Bluetooth da ESP identificada neste teste. O receptor envia PING, verifica PONG, recebe durante 60 segundos e encerra o canal. Mostra amostras, salva objetos JSON e imprime contagens de quadros, IMU válida, erros e lacunas. **Depois que ele termina, aparecer “desconectado” nos Ajustes do Mac é esperado.** Só olhar “conectado” nos Ajustes não confirma que os dados chegaram.

O USB pode continuar conectado como alimentação: o receptor acima não abre a USB; os dados chegam via rádio. Mantenha a bateria dos motores desligada neste teste. Para interromper antes, use Ctrl+C; este firmware não contém navegação ou comando de movimento.

O binário foi compilado para este Mac Apple Silicon. O fonte `receber_mac.m` está junto dele. Para recompilar, se necessário:

```bash
clang -fobjc-arc -framework Foundation -framework IOBluetooth receber_mac.m -o receber_mac
```

Se não estiver pareado, use Ajustes do Sistema → Bluetooth → `MicroMouse_BT_Test` para parear. Conceda permissão de Bluetooth ao aplicativo, se o macOS solicitar. Evite abrir dois receptores simultaneamente. A conexão útil é mantida pelo receptor enquanto ele está capturando.

### Receptor Python alternativo

`receber_telemetria.py` usa a porta serial SPP do sistema e passou seus quatro testes de enquadramento/validação. Ele funcionou na primeira captura, mas a reconexão dessa porta falhou neste Mac; portanto prefira o receptor nativo acima.

```bash
/Users/gustavo/.platformio/penv/bin/python receber_telemetria.py --seconds 20 --capture captura-python.jsonl
```

Os resumos comprovam apenas cada captura realizada; não garantem ausência universal de perdas. Nenhum receptor inicia servidor, banco de dados ou dashboard.

## Gravar novamente o teste

Mantenha a bateria dos motores desligada; somente USB. A IMU calibra no início: mantenha o robô imóvel.

No VS Code/PlatformIO, abra:

```text
/Users/gustavo/UnB/2026.2_PI1_Grupo2_Lui/src/firmwareTest/BluetoothTelemetry
```

Escolha Build e depois Upload. Pelo terminal:

```bash
cd /Users/gustavo/UnB/2026.2_PI1_Grupo2_Lui/src/firmwareTest/BluetoothTelemetry
/Users/gustavo/.platformio/penv/bin/pio run
/Users/gustavo/.platformio/penv/bin/pio run -t upload --upload-port /dev/cu.usbserial-0001
```

A porta USB pode mudar; confira `pio device list`. Feche qualquer monitor que esteja usando a USB antes de gravar. A gravação substitui o firmware na placa, mas não apaga os arquivos do firmware de navegação.

## Monitor USB e comandos

USB a 115200 baud, porta `/dev/cu.usbserial-0001`:

| Comando | Ação neste firmware |
|---|---|
| `s` ou `h` | Estado de conexão, streaming e contador |
| `p` ou `x` | Pausar somente envio de dados |
| `r` | Retomar envio |

Esses comandos não movimentam o robô. O comando `g` de navegação não existe neste teste. Pelo enlace Bluetooth, `PING identificador` seguido de Enter retorna um objeto `pong` com o mesmo identificador; o receptor faz isso automaticamente.

## Voltar ao labirinto

Para restaurar a missão, depois da conferência elétrica e dos ensaios físicos, grave o projeto principal em `/Users/gustavo/UnB/2026.2_PI1_Grupo2_Lui/src/firmware`. A imagem de navegação não é restaurada ao fechar o receptor ou reiniciar a ESP.

## Referências

- [BluetoothSerial/SPP oficial da Espressif](https://docs.espressif.com/projects/arduino-esp32/en/latest/api/bluetooth.html)
- [API nativa IOBluetooth — Apple](https://developer.apple.com/documentation/IOBluetooth/IOBluetoothDevice)
- [Conectar um dispositivo Bluetooth ao Mac — Apple](https://support.apple.com/en-euro/guide/mac-help/blth1004/27/mac/27)


## Integração com o site

O backend agora reconhece `bt_telemetry_test` em um modo de monitoramento sem persistir corrida. O site exibe a IMU real, sequência e atualização pelo WebSocket. Use o [guia do monitor](../../backend/BLUETOOTH.md); não é necessário regravar esta aplicação para abrir o painel. O backend usa o receptor nativo em modo contínuo/JSON e passa a ser o cliente Bluetooth, portanto feche receptores separados antes de iniciá-lo.


### Compensação de yaw em repouso

A calibração ocorre com Bluetooth já iniciado: 2 s de aquecimento e 1000 amostras espaçadas por 5 ms. Deixe o robô imóvel na inicialização. O estimador DiagnosticYaw mantém o ângulo após 1 s com giro residual abaixo de 0,25°/s, aceleração próxima de 1 g e pequenas variações entre amostras; nesse estado adapta lentamente o offset residual. O gyro_z_dps transmitido continua sendo a leitura do driver após calibração inicial, para não esconder o resíduo medido. A integração de yaw usa a compensação adicional. Leituras inválidas ou intervalos acima de 100 ms não são integrados.

A IMU sozinha não distingue rotação constante extremamente lenta de bias. Este filtro está limitado ao diagnóstico com motores desligados e não altera o driver compartilhado nem o controle de navegação. Não fornece orientação absoluta. Depois de gravar, valide tanto o repouso quanto um giro manual antes de considerar o ângulo calibrado.

Teste de cálculo no Mac (na pasta BluetoothTelemetry): `clang++ -std=c++17 tests/test_relative_yaw.cpp -o /tmp/test_relative_yaw && /tmp/test_relative_yaw`. Usa sequências sintéticas; não substitui o teste físico de giro.
