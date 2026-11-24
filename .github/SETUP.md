# Guia de Configuração 

## 1. Arquitetura resumida

### 1.1 Firmware embarcado

O firmware utiliza uma **Arquitetura em Camadas (Layered Architecture)** orientada a eventos/tarefas.

A divisão principal é:

```text
HAL / Drivers
      ↓
Controle e Navegação
      ↓
Comunicação e Armazenamento
```

### 1.2 Central de controle

O backend/frontend segue o padrão **MVC (Model-View-Controller)**:

```text
React frontend
      ↓
HTTP / WebSocket
      ↓
Backend Node.js
      ↓
Controllers
      ↓
Services
      ↓
Prisma
      ↓
PostgreSQL
```

### 1.3 Comunicação

A comunicação recomendada entre as partes é:

```text
ESP32 ── WebSocket ──> Backend
                         │
                         └── WebSocket ──> frontend
```

O ESP32 funciona como cliente de comunicação e o backend como servidor WebSocket da central.

---

## 2. Estrutura do repositório

A estrutura recomendada é:

```text
micromouse/
├── src/
|   ├── backend/
|   │   ├── prisma/
|   │   ├── src/
|   │   │   ├── controllers/
|   │   │   ├── services/
|   │   │   ├── routes/
|   │   │   ├── models/
|   │   │   ├── ws/
|   │   │   ├── lib/
|   │   │   ├── app.ts
|   │   │   └── server.ts
|   │   ├── .env
|   │   ├── prisma7.config.ts
|   │   ├── package.json
|   │   └── tsconfig.json
|   |
|   ├── firmware/
|   │   ├── include/
|   │   ├── lib/
|   │   ├── src/
|   │   │   ├── hal/
|   │   │   ├── control/
|   │   │   ├── navigation/
|   │   │   ├── communication/
|   │   │   ├── storage/
|   │   │   ├── tasks/
|   │   │   └── main.cpp
|   │   └── platformio.ini
|   |
|   └── frontend/
|       ├── src/
|       │   ├── components/
|       │   ├── pages/
|       │   ├── services/
|       │   ├── hooks/
|       │   ├── App.tsx
|       │   └── main.tsx
|       ├── package.json
|       └── vite.config.ts
├── docker-compose.yml
├── .gitignore
├── README.md
└── CONTRIBUTING.md

```

### Responsabilidade dos diretórios

| Diretório | Responsabilidade |
|---|---|
| `src/backend` | API, WebSocket, regras de negócio e persistência |
| `src/backend/prisma` | Schema e migrations do banco |
| `src/frontend` | Interface web de monitoramento |
| `firmware/src/hal` | Drivers de hardware |
| `firmware/src/control` | Controle PID, odometria e lógica de controle |
| `firmware/src/navigation` | Mapeamento e navegação/Floodfill |
| `firmware/src/communication` | Wi-Fi, WebSocket e comunicação de alto nível |
| `firmware/src/storage` | Persistência local no ESP32 |
| `firmware/src/tasks` | Tasks do FreeRTOS |
| `docker-compose.yml` | Infraestrutura local do PostgreSQL |

---

## 3. Pré-requisitos

Antes de começar, instale:

| Ferramenta | Versão de referência |
|---|---:|
| Node.js | `24.21.0` |
| npm | `11.19.0` |
| PlatformIO Core | `6.2.x` |
| Arduino ESP32 Core | `3.3.11` |
| PostgreSQL | `18.6` |
| Docker Engine | `29.8.x` |
| Docker Compose | `5.6.x` |
| Prisma CLI | `7.10.0` |

Também é necessário ter acesso ao hardware e aos cabos/programadores utilizados para o ESP32 quando a contribuição envolver firmware.

### Verificação do ambiente

```bash
node -v
npm -v
docker --version
docker compose version
pio --version
```

O resultado deve indicar versões compatíveis com as versões de referência acima.

---

## 4. Clonar o projeto

Clone o repositório e entre na pasta:

```bash
git clone https://github.com/fcte-pi1/2026.2_PI1_Grupo2_Lui.git # ou "git clone git@github.com:fcte-pi1/2026.2_PI1_Grupo2_Lui.git"
cd 2026.2_PI1_Grupo2_Lui/
```

---

## 5. Configuração do PostgreSQL

O PostgreSQL deve ser executado localmente por Docker Compose.

Na raiz do projeto:

```bash
docker compose up -d
```

Verifique os containers:

```bash
docker compose ps
```

O serviço esperado é:

```text
micromouse-postgres
```

### Parar o banco

```bash
docker compose down
```

## 6. Configuração do backend

Entre no backend:

```bash
cd src/backend
```

Instale as dependências:

```bash
npm install
```

O backend utiliza:

- TypeScript;
- Node.js;
- Express;
- `ws` para WebSocket;
- Prisma;
- PostgreSQL.

### Arquivo `.env`

Crie:

```text
apps/backend/.env
```

Exemplo:

```env
DATABASE_URL="postgresql://micromouse:<SENHA>@localhost:5433/micromouse?schema=public"
PORT=3000
```

> Nunca envie o `.env` para o Git.

### Banco e Prisma

Depois de iniciar o PostgreSQL:

```bash
npx prisma migrate dev
npx prisma generate
```

Para visualizar os dados:

```bash
npx prisma studio
```

### Executar o backend

```bash
npm run dev
```

Teste:

```text
http://localhost:3000/health
```

A resposta esperada é semelhante a:

```json
{
  "status": "ok",
  "service": "micromouse-backend"
}
```

---

## 7. Modelo de dados

O banco representa quatro entidades principais:

```text
SESSAO_NAVEGACAO
       │
       ├── CELULA_LABIRINTO
       ├── REGISTRO_TELEMETRIA
       └── LOG_EVENTO
```

### `SESSAO_NAVEGACAO`

Representa um percurso do robô.

Principais informações:

- Identificador da sessão;
- Data/hora de início;
- Status de conclusão;
- Tempo total da execução.

### `CELULA_LABIRINTO`

Representa uma célula mapeada durante a navegação.

Principais informações:

- Coordenadas `X` e `Y`;
- Paredes norte/sul/leste/oeste;
- Estado de visita;
- Sessão relacionada.

### `REGISTRO_TELEMETRIA`

Armazena as amostras da execução.

Principais informações:

- Timestamp;
- Tensão da bateria;
- Corrente da bateria;
- Velocidade esquerda;
- Velocidade direita;
- Ângulo de orientação;
- Sessão relacionada.

### `LOG_EVENTO`

Registra acontecimentos relevantes.

Exemplos:

```text
INFO
WARN
ERROR
SUCCESS
```

Exemplos de mensagens:

```text
Linha de Chegada Reconhecida
Motores Interrompidos
Falha de Comunicação
```

---

## 8. Regras para migrations

Alterações no modelo do banco devem ser feitas através do Prisma.

### Criar uma migration

Depois de modificar `schema.prisma`:

```bash
npx prisma migrate dev --name descricao-da-alteracao
```

Exemplo:

```bash
npx prisma migrate dev --name add_session_status
```

### Regra importante

Não altere manualmente tabelas no PostgreSQL para implementar mudanças que deveriam estar no `schema.prisma`.

A alteração deve seguir:

```text
schema.prisma
     ↓
migration
     ↓
PostgreSQL
```

Sempre envie a migration criada junto com o código que depende dela.

---

## 9. Configuração do frontend

Entre no frontend:

```bash
cd src/frontend
```

Instale as dependências:

```bash
npm install
```

O frontend utiliza:

- React;
- TypeScript/JavaScript;
- Vite;
- TailwindCSS;
- Chart.js;
- HTML5 Canvas;
- WebSocket.

### Executar o frontend

```bash
npm run dev
```

O Vite exibirá o endereço local, normalmente:

```text
http://localhost:5173
```

---

## 10. Estrutura do frontend

Os componentes devem ser organizados por responsabilidade.

Exemplo:

```text
src/
├── components/
│   ├── MazeGrid.tsx
│   ├── RobotStatus.tsx
│   ├── TelemetryChart.tsx
│   ├── BatteryStatus.tsx
│   └── EventLog.tsx
│
├── pages/
│   ├── dashboard.tsx
│   └── Session.tsx
│
├── services/
│   ├── api.ts
│   └── websocket.ts
│
└── hooks/
    └── useTelemetry.ts
```

### Responsabilidades

**`components/`**

Componentes visuais reutilizáveis.

**`pages/`**

Composição das telas.

**`services/`**

Comunicação com API e WebSocket.

**`hooks/`**

Estado e comportamento reutilizável do React.

---

## 11. Configuração do firmware

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

## 12. Estrutura do firmware

O firmware deve seguir as camadas definidas na arquitetura.

### `hal/`

Drivers de baixo nível:

```text
imu_mpu6500
ultrasonic_us100
motor_drv8833
encoder_n20
ina226
```

### `control/`

Controle do movimento:

```text
PID
odometria
controle dos motores
```

### `navigation/`

Lógica de navegação:

```text
mapeamento
matriz do labirinto
Floodfill
decisão de movimento
```

### `communication/`

Comunicação:

```text
Wi-Fi
WebSocket
serialização JSON
telemetria
```

### `storage/`

Armazenamento local:

```text
Preferences
LittleFS
logs
configurações
```

### `tasks/`

Tarefas FreeRTOS:

```text
Task_Sensors
Task_PID_Control
Task_Navigation
Task_Comm
```

---

## 13. Tasks do FreeRTOS

A arquitetura estabelece as seguintes tarefas:

| Task | Tipo | Período/Disparo | Prioridade |
|---|---|---:|---:|
| `Task_Sensors` | Periódica | 10 ms | Alta |
| `Task_PID_Control` | Periódica | 5 ms | Máxima |
| `Task_Navigation` | Baseada em eventos | Centro da célula | Média |
| `Task_Comm` | Assíncrona | Conforme comunicação | Baixa |

### Regra de implementação

Tarefas de tempo real não devem ser bloqueadas por operações demoradas de comunicação, armazenamento ou interface.

A lógica de sensores e controle deve continuar responsiva mesmo quando houver atraso na comunicação Wi-Fi.

---

## 14. Hardware contemplado

A arquitetura considera:

- ESP32 DevKit V1;
- Ponte H DRV8833;
- 2 motores N20 com encoders;
- Sensores ultrassônicos US-100;
- IMU MPU6500;
- INA226;
- Regulador Buck MP1584.

Antes de alterar GPIOs, barramentos ou interfaces, confira o documento de arquitetura e o diagrama de implantação.

> A documentação apresenta uma divergência entre a quantidade de sensores ultrassônicos indicada na Visão de Processos e na Visão de Implantação. Antes de implementar mudanças nessa parte, confirme a definição vigente do projeto.

---
