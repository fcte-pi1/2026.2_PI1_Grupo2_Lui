# Micromouse — PlatformIO / ESP32

Projeto adaptado do sketch `codigo_do_labirinto.ino` para a estrutura C++ do PlatformIO.

## Placa e ambiente

- Placa: ESP32 DevKit V1 (módulo ESP32 clássico, por exemplo ESP32-WROOM-32)
- Board ID do PlatformIO: `esp32dev`
- Framework: Arduino
- Monitor Serial: `115200` baud
- Bibliotecas externas: nenhuma. `Wire.h` e os recursos usados são fornecidos pelo framework Arduino para ESP32.

## Estrutura

```text
MicromousePlatformIO/
├── platformio.ini
├── README.md
├── .gitignore
└── src/
    └── main.cpp
```

## Como abrir e executar no VS Code

1. Instale a extensão **PlatformIO IDE** no VS Code.
2. Extraia esta pasta e abra a pasta `MicromousePlatformIO` (a que contém `platformio.ini`) no VS Code.
3. Conecte o ESP32 por USB e selecione a porta serial, caso o PlatformIO não a detecte automaticamente.
4. Na barra do PlatformIO, use **Build** para compilar e **Upload** para gravar. Também é possível usar o terminal integrado:

   ```bash
   pio run
   pio run --target upload
   pio device monitor
   ```

5. Abra o monitor serial em `115200` baud. O programa aguarda o botão de Start no GPIO 13.

Se a gravação não iniciar automaticamente, mantenha pressionado o botão **BOOT** da placa quando o PlatformIO começar a conectar e solte-o quando a gravação começar.

## Observações importantes de hardware

- Os pinos de Echo definidos no código (GPIO 35, 36 e 39) são somente entradas e são adequados para leitura de sensores ultrassônicos.
- **Atenção ao nível lógico:** os GPIOs do ESP32 não são tolerantes a 5 V. Se o Echo do US-100/HC-SR04 fornecer 5 V, use divisor resistivo ou conversor de nível para reduzir o sinal a 3,3 V antes de conectá-lo ao ESP32.
- Os sensores devem estar configurados no modo de trigger/echo. O modo UART do US-100 exige outra implementação.
- Confira a pinagem real do seu módulo e do driver DRV8833 antes de energizar os motores. O código considera o pino GPIO 5 como habilitação (`nSLEEP`/EEP, conforme a placa do driver).
- O algoritmo foi preservado na adaptação. Antes de testar com as rodas no chão, faça o primeiro teste com o robô suspenso e confira o sentido dos motores, os encoders, os sensores e o botão de Start.
- As rotinas de giro e deslocamento usam laços de controle sem timeout geral. Uma leitura inválida da IMU ou falha de encoder pode impedir a rotina de terminar; valide os sensores e mantenha um meio físico de cortar a alimentação dos motores durante os testes.

## Compatibilidade do PWM

O código mantém a seleção condicional da API LEDC:
- Arduino-ESP32 2.x: `ledcSetup`, `ledcAttachPin` e `ledcWrite` por canal.
- Arduino-ESP32 3.x: `ledcAttach` e `ledcWrite` por pino.
