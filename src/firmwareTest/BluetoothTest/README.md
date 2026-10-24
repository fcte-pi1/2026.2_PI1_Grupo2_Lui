# BluetoothTest — ESP32 DevKit V1

Teste isolado de **Bluetooth Classic (Serial Port Profile / SPP)**. A ESP32 anuncia o nome `MicroMouse_BT_Test`, envia mensagens `PING` periodicamente depois de conectar e devolve `ECHO: <mensagem>` quando recebe uma linha do outro dispositivo.

> Este teste usa Bluetooth Classic SPP, não Bluetooth Low Energy (BLE). Ele é direcionado à ESP32 DevKit V1 baseada no ESP32 original. Placas como ESP32-C3 e ESP32-S3 não oferecem o mesmo suporte a Bluetooth Classic SPP.

## Gravar e iniciar

Na raiz do repositório, execute:

```bash
cd src/firmwareTest/BluetoothTest
pio run
pio run --target upload --upload-port /dev/ttyUSB0
pio device monitor --port /dev/ttyUSB0 --baud 115200
```

Ajuste a porta serial se necessário (`/dev/ttyACM0` ou `COMx`). Também é possível abrir diretamente esta pasta no VS Code/PlatformIO e usar **Build**, **Upload** e **Monitor**.

## Testar com celular ou computador

1. Deixe o monitor serial USB aberto a 115200 baud.
2. Ative o Bluetooth do celular e procure `MicroMouse_BT_Test` na lista de dispositivos. Pareie, se solicitado.
3. Abra um aplicativo de terminal que suporte **Bluetooth Classic SPP** e conecte-se a `MicroMouse_BT_Test`. No Android, pode ser utilizado um terminal serial Bluetooth compatível com SPP.
4. Confira no terminal Bluetooth as mensagens `PING 1`, `PING 2` etc.
5. Envie `teste 123` e pressione ENTER. A ESP32 deve responder `ECHO: teste 123`; o monitor USB também deve registrar a mensagem recebida.
6. Desconecte e reconecte. O monitor USB deve registrar a transição de conexão.

## Como interpretar

- A mensagem de inicialização no monitor USB indica apenas que a biblioteca iniciou o rádio; **não comprova que outro dispositivo se conectou**.
- O log `[PASS] Conexao Bluetooth SPP estabelecida` confirma que um cliente SPP se conectou.
- Receber `PING` e obter a resposta `ECHO` confirma comunicação nos dois sentidos.
- Se o nome não aparecer, confirme que a placa é uma ESP32 original com suporte a Bluetooth Classic, reinicie a placa e procure novamente. Se aparecer mas não conectar, remova o pareamento antigo nos dois dispositivos e tente novamente.
- Não conecte fios nem periféricos externos para este teste; a comunicação é sem fio.

Ao terminar, feche o monitor e grave novamente o projeto principal em `src/firmware/` para restaurar o firmware do robô.
