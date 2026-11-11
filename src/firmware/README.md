# Firmware MicroMouse — Arduino IDE

O firmware está organizado como um sketch da **Arduino IDE** para a placa **ESP32
DevKit V1 (ESP32 clássico / ESP32-WROOM-32)**. O projeto também mantém o
`platformio.ini` para compilação e execução dos testes nativos pelo PlatformIO.

## 1. Preparar a Arduino IDE

1. Instale a Arduino IDE 2.x.
2. Em **Arquivo → Preferências**, adicione o índice de placas ESP32 no campo
   **URLs adicionais para Gerenciadores de Placas**:

   ```text
   https://espressif.github.io/arduino-esp32/package_esp32_index.json
   ```

3. Abra **Ferramentas → Placa → Gerenciador de Placas**, pesquise por
   **esp32 by Espressif Systems** e instale a versão **3.3.11**, alinhada à
   versão de referência do projeto em `.github/SETUP.md`.
4. Não é necessária biblioteca externa adicional no Library Manager.
   `BluetoothSerial`, `Wire`, `LittleFS` e as APIs NVS são fornecidas pelo core
   ESP32 instalado no passo anterior.

## 2. Abrir o sketch

Abra o arquivo `src/firmware/firmware.ino` pela Arduino IDE. A pasta do sketch
precisa continuar se chamando `firmware`, assim como o arquivo principal
`firmware.ino`.

A implementação está dividida em módulos C++ sob `src/` (`hal`, `control`,
`navigation`, `communication`, `config` e `common`). A Arduino IDE compila
recursivamente os arquivos C/C++ dessa pasta. As funções `setup()` e `loop()`
continuam implementadas em `src/main.cpp`.

## 3. Selecionar placa e parâmetros

Em **Ferramentas**, selecione:

| Opção | Configuração |
|---|---|
| Placa | **ESP32 Dev Module** |
| Upload Speed | `115200` |
| CPU Frequency | `240MHz (WiFi/BT)` ou a opção equivalente disponível |
| Flash Size | `4MB (32Mb)` |
| Partition Scheme | `Default 4MB with spiffs` ou a opção padrão equivalente |
| Core Debug Level | `None` |

Conecte a ESP32 por USB, selecione a porta em **Ferramentas → Porta** e confirme
que a placa escolhida é a ESP32 clássica DevKit V1, não uma variante ESP32-S2,
ESP32-S3 ou ESP32-C3. O transporte do firmware utiliza Bluetooth Classic SPP.

## 4. Configuração local (opcional)

Os valores padrão permitem iniciar o firmware sem criar arquivo adicional. Para
personalizar o nome Bluetooth ou a calibração do INA226:

1. Copie `src/config/network_config.local.example.hpp`.
2. Salve a cópia como `src/config/network_config.local.hpp`.
3. Edite os valores necessários. Esse arquivo é ignorado pelo Git.

O nome Bluetooth padrão anunciado pela ESP32 é `MicroMouse`. A comunicação é
Bluetooth Classic SPP (porta serial), não BLE.

## 5. Compilar e gravar

1. Clique em **Verificar** para compilar.
2. Com a ESP32 conectada, clique em **Carregar**.
3. Se a placa não entrar automaticamente no modo de gravação, mantenha o botão
   **BOOT** pressionado quando a IDE iniciar o envio e solte-o quando a gravação
   começar.
4. Abra o **Monitor Serial** em `115200` baud para acompanhar a inicialização,
   sensores, motores e estado da conexão Bluetooth.

## 6. Organização dos arquivos

```text
firmware/
├── firmware.ino             # Sketch principal exigido pela Arduino IDE
├── src/                     # Código C/C++ compilado recursivamente
│   ├── common/
│   ├── communication/
│   ├── config/
│   ├── control/
│   ├── hal/
│   ├── navigation/
│   └── main.cpp              # setup() e loop()
├── include/                 # Headers de compatibilidade para projetos legados
├── test/                    # Testes unitários do PlatformIO
└── platformio.ini            # Configuração alternativa do PlatformIO
```

Os arquivos de `include/` são apenas encaminhadores para os headers mantidos em
`src/`. Essa compatibilidade preserva os caminhos utilizados pelos projetos
independentes em `src/firmwareTest/`, sem exigir alterações neles.

## 7. PlatformIO e testes nativos

A estrutura continua incluindo `platformio.ini`. Para os testes nativos de
regras de navegação e lógica sem hardware, abra `src/firmware/` no PlatformIO e
execute:

```bash
pio test -e native
```

Para gravar pelo PlatformIO, o ambiente `esp32dev` continua disponível:

```bash
pio run -e esp32dev
pio run -e esp32dev --target upload
```
