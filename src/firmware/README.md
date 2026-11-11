# Como gravar o firmware na placa (ESP32)

Este guia mostra, passo a passo, como passar o código do MicroMouse do computador
para a placa **ESP32 DevKit V1** usando a **Arduino IDE**. Não é preciso ter
experiência com Arduino.

**Você vai precisar de:**

- um computador com o repositório baixado;
- a placa ESP32 DevKit V1;
- um cabo USB **de dados** 

> **Antes de começar:** quando a gravação termina, o robô liga e **começa a
> andar sozinho** em poucos segundos. Deixe o robô **parado e com as rodas sem
> tocar a mesa** ou já colocado no labirinto.

---

## Parte 1 — Preparar o computador (só na primeira vez)

Se você já fez esta parte antes, pule para a [Parte 2](#parte-2--gravar-o-código-na-placa).

### Passo 1: Instalar a Arduino IDE

1. Acesse <https://www.arduino.cc/en/software>.
2. Baixe a **Arduino IDE 2.x** para o seu sistema e instale normalmente
   (Avançar → Avançar → Concluir).
3. Abra a Arduino IDE.

### Passo 2: Ensinar a Arduino IDE a reconhecer a ESP32

1. No menu de cima, clique em **Arquivo → Preferências**.
2. Procure o campo **URLs adicionais para Gerenciadores de Placas**.
3. Cole este endereço nele:

   ```text
   https://espressif.github.io/arduino-esp32/package_esp32_index.json
   ```

   Se já houver outro endereço no campo, coloque uma vírgula depois dele e cole
   este em seguida.
4. Clique em **OK**.

### Passo 3: Instalar o pacote da ESP32

1. No menu de cima, clique em **Ferramentas → Placa → Gerenciador de Placas**.
   Um painel vai abrir na lateral esquerda.
2. Na caixa de busca, digite `esp32`.
3. Encontre o item **esp32 by Espressif Systems**.
4. Na lista de versões desse item, escolha **3.3.11**. Use exatamente essa
   versão, porque com outras o código pode não compilar.
5. Clique em **Instalar** e espere terminar. Pode demorar alguns minutos.

Você **não precisa** instalar nenhuma biblioteca extra. Tudo o que o código usa
já vem nesse pacote.

### Passo 4: Instalar o driver USB (só se a placa não aparecer no Passo 7)

A maioria dos computadores reconhece a placa sozinha. Se no Passo 7 nenhuma
porta nova aparecer, instale o driver do chip USB da sua placa. Ele vem escrito
no chip pequeno perto da entrada USB.

- Chip **CP2102** ou **CP210x**: instale o driver da Silicon Labs
  (<https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers>).
- Chip **CH340**: instale o driver da WCH
  (<https://www.wch-ic.com/downloads/CH341SER_EXE.html>).

Depois de instalar, desconecte e conecte a placa de novo.

---

## Parte 2 — Gravar o código na placa

### Passo 5: Abrir o projeto

1. Na Arduino IDE, clique em **Arquivo → Abrir…**.
2. Navegue até a pasta do repositório e abra o arquivo:

   ```text
   src/firmware/firmware.ino
   ```

3. O arquivo aberto é quase vazio. **Isso é normal.** O código de verdade fica
   na pasta `src/firmware/src/`, e a Arduino IDE o encontra sozinha.

> Não renomeie a pasta `firmware` nem o arquivo `firmware.ino`. A Arduino IDE só
> funciona se os dois tiverem o mesmo nome.

### Passo 6: Escolher a placa

1. Clique em **Ferramentas → Placa → esp32 → ESP32 Dev Module**.
2. Ainda em **Ferramentas**, confira se as opções abaixo estão assim. Se alguma
   estiver diferente, clique nela e troque.

   | Opção em Ferramentas | Deve estar em |
   |---|---|
   | Upload Speed | `115200` |
   | CPU Frequency | `240MHz (WiFi/BT)` |
   | Flash Size | `4MB (32Mb)` |
   | Partition Scheme | `Default 4MB with spiffs` |
   | Core Debug Level | `None` |

### Passo 7: Conectar a placa e escolher a porta

1. Com a placa **desconectada**, abra **Ferramentas → Porta** e veja quais portas
   aparecem na lista (pode até estar vazia).
2. Conecte a ESP32 no computador pelo cabo USB. Um LED da placa deve acender.
3. Abra **Ferramentas → Porta** de novo. A porta **que apareceu agora** é a da
   placa. Clique nela para selecionar.
   - No Windows, o nome é parecido com `COM3` ou `COM5`.
   - No Linux, é parecido com `/dev/ttyUSB0`.
   - No macOS, é parecido com `/dev/cu.usbserial-...`.

Se nenhuma porta nova aparecer, troque o cabo USB ou instale o driver (Passo 4).

### Passo 8: Gravar

1. Clique no botão **Carregar**: a **seta para a direita (→)** no canto superior
   esquerdo.
2. A Arduino IDE primeiro compila o código e depois grava na placa. A
   **primeira vez demora alguns minutos**, e as próximas são mais rápidas.
3. Acompanhe a parte de baixo da janela:
   - Se aparecer `Connecting....____....` e ficar parado, **aperte e segure o
     botão BOOT** da placa. Solte quando começar a aparecer a porcentagem
     (`Writing at ... (10 %)`).
   - Quando aparecer **`Hard resetting via RTS pin...`**, a gravação terminou com
     sucesso.

### Passo 9: Conferir se deu certo

1. Clique em **Ferramentas → Monitor Serial**.
2. No canto direito do monitor, selecione **115200 baud**.
3. Aperte o botão **EN** (ou **RST**) da placa para reiniciá-la e ver as
   mensagens desde o começo. Mantenha o robô **parado** enquanto aparecer
   `Calibrando IMU`.
4. Se tudo estiver certo, as mensagens vão ser parecidas com estas:

   ```text
   [OK] MPU6500 respondeu no barramento I2C.
   Calibrando IMU. Mantenha o robô parado.
   HU-10: cliente Bluetooth iniciado; ...
   Labirinto 4x4; início=(...); objetivo=(...).
   Navegação autônoma iniciada; sem intervenção manual durante a tentativa.
   ```

- `HU-09: INA226 ausente/não calibrado` é só um aviso. O robô funciona
  normalmente, só não mostra o nível da bateria.
- Qualquer mensagem começando com **`Falha`** indica um problema. Veja a tabela
  [Deu erro?](#deu-erro).

Pronto, o código está na placa. 🎉 Para gravar uma nova versão depois, basta
repetir a **Parte 2**.

---

## Organização dos arquivos

```text
firmware/
├── firmware.ino             # Sketch principal exigido pela Arduino IDE
├── src/                     # Todo o código C/C++ (compilado automaticamente)
│   ├── common/              # Tipos compartilhados entre os módulos
│   ├── communication/       # Logger e telemetria via Bluetooth
│   ├── config/              # Pinos da placa, labirinto e configuração local
│   ├── control/             # PID, odometria e controle de movimento
│   ├── hal/                 # Drivers de motores, encoders, IMU, INA226 e armazenamento
│   ├── navigation/          # Mapa, solver, percepção de paredes e navegação autônoma
│   └── main.cpp             # setup() e loop()
├── include/                 # Headers de compatibilidade usados por src/firmwareTest/
├── test/                    # Testes unitários do PlatformIO
└── platformio.ini           # Configuração alternativa do PlatformIO
```

O código do robô fica todo em `src/`. A Arduino IDE compila essa pasta inteira
sozinha, então o `firmware.ino` existe só porque a IDE exige um arquivo com o
mesmo nome da pasta.

Os arquivos de `include/` não têm código próprio: cada um apenas aponta para o
arquivo correspondente em `src/`. Eles existem para que os projetos de teste de
hardware em `src/firmwareTest/` continuem compilando sem alterações. **Ao alterar
o código, edite sempre os arquivos em `src/`.**

---

## PlatformIO e testes

O projeto também pode ser compilado com o PlatformIO, pelo `platformio.ini`.
Para rodar os testes automáticos no computador, sem a placa:

1. Instale o PlatformIO com `python -m pip install -U platformio`, ou instale a
   extensão **PlatformIO IDE** no VS Code.
2. Instale um compilador C/C++. No Windows, rode
   `winget install -e --id BrechtSanders.WinLibs.POSIX.UCRT` e depois
   **feche todas as janelas do VS Code/terminal** e abra de novo. No Linux, use
   `build-essential`. No macOS, use `xcode-select --install`.
3. Na pasta `src/firmware/`, rode:

   ```bash
   pio test -e native                     # testes
   pio run -e esp32dev --target upload    # gravar na placa pelo PlatformIO
   ```

   Se o comando `pio` não for reconhecido, use `python -m platformio` no lugar
   dele.
