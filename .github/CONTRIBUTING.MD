# Guia de contribuição

## Estrutura do repositório

**OBS.:** Estrutura sujeita a mudanças

```

Repo/
├── docs/
│   ├── Arquitetura.md
│   └── figs/
│       └── Projeto_conceitual/
│           ├── VisaoDeProcessos.png
│           └── VisaoDeImplantaçao.png
└── src/
    ├── backend/
    │   ├── prisma/                  # Schema e migrações do banco relacional (SQLite / PostgreSQL)
    │   │   └── schema.prisma
    │   ├── src/
    │   │   ├── controllers/         # Camada Controller (MVC): Handlers das rotas HTTP e WebSockets
    │   │   │   ├── SessionController.ts
    │   │   │   ├── TelemetryController.ts
    │   │   │   └── LogController.ts
    │   │   ├── models/              # Camada Model (MVC) / Entidades do Banco de Dados
    │   │   │   ├── SessaoNavegacao.ts
    │   │   │   ├── CelulaLabirinto.ts
    │   │   │   ├── RegistroTelemetria.ts
    │   │   │   └── LogEvento.ts
    │   │   ├── services/            # Regras de negócio e processamento de telemetria
    │   │   │   ├── TelemetryProcessor.ts
    │   │   │   └── MazeService.ts
    │   │   ├── websocket/           # Servidor WebSockets (ws) para recepção de dados em tempo real
    │   │   │   └── wsServer.ts
    │   │   ├── routes/              # Definidor de rotas Express.js
    │   │   │   └── apiRoutes.ts
    │   │   └── index.ts             # Entry point do servidor Node.js/Express
    │   ├── package.json
    │   └── tsconfig.json
    ├── firmware/
    │   ├── include/                 # Arquivos de cabeçalho (.h / .hpp) organizados por camadas
    │   │   ├── drivers/             # Módulo 1: HAL / Drivers de Hardware (Baixo Nível)
    │   │   │   ├── mpu6500.h        # Driver IMU
    │   │   │   ├── us100.h          # Driver Ultrassônico
    │   │   │   ├── drv8833.h        # Driver Ponte H
    │   │   │   ├── encoder.h        # Driver de Odometria
    │   │   │   └── ina226.h         # Driver de Potência
    │   │   ├── control/             # Módulo 2: Controle e Navegação (Médio Nível)
    │   │   │   ├── pid.h            # Malha de Controle PID
    │   │   │   └── floodfill.h      # Algoritmo Mapeador e Buscador
    │   │   ├── comm/                # Módulo 3: Comunicação e Armazenamento (Alto Nível)
    │   │   │   ├── storage.h        # Gerenciador NVS / LittleFS
    │   │   │   └── ws_client.h      # Cliente WebSockets / Wi-Fi
    │   │   └── tasks/               # Tarefas FreeRTOS (Visão de Processos)
    │   │       ├── task_sensors.h
    │   │       ├── task_pid.h
    │   │       ├── task_navigation.h
    │   │       └── task_comm.h
    │   ├── src/                     # Implementação (.cpp)
    │   │   ├── drivers/
    │   │   │   ├── mpu6500.cpp
    │   │   │   ├── us100.cpp
    │   │   │   ├── drv8833.cpp
    │   │   │   ├── encoder.cpp
    │   │   │   └── ina226.cpp
    │   │   ├── control/
    │   │   │   ├── pid.cpp
    │   │   │   └── floodfill.cpp
    │   │   ├── comm/
    │   │   │   ├── storage.cpp
    │   │   │   └── ws_client.cpp
    │   │   ├── tasks/
    │   │   │   ├── task_sensors.cpp     # Core 0: Amostragem IMU, US-100, INA226 (10ms)
    │   │   │   ├── task_pid.cpp         # Core 1: Encoders e Controle PWM DRV8833 (5ms)
    │   │   │   ├── task_navigation.cpp  # Core 1: Atualização da grade e decisão Floodfill
    │   │   │   └── task_comm.cpp        # Core 0: Envio JSON via WebSockets e NVS
    │   │   └── main.cpp                 # Inicialização do FreeRTOS e pinout ESP32 DevKit V1
    │   ├── data/                    # Arquivos gravados na Flash interna via LittleFS
    │   └── platformio.ini           # Configurações do firmware (Framework Arduino ESP32 / Libs)
    └── frontend/
        ├── public/
        ├── src/
        │   ├── components/          # Componentes visuais da interface (View)
        │   │   ├── MazeCanvas.tsx   # Renderização do labirinto (HTML5 Canvas API)
        │   │   ├── TelemetryChart.tsx # Gráficos de telemetria em tempo real (Chart.js)
        │   │   ├── BatteryWidget.tsx # Leitura de tensão e corrente
        │   │   └── EventLogs.tsx    # Painel de logs de execução
        │   ├── services/            # Cliente de comunicação com backend
        │   │   ├── wsService.ts     # Conexão WebSocket para telemetria
        │   │   └── api.ts           # Chamadas HTTP/REST
        │   ├── types/               # Tipagens TypeScript baseadas no MER do sistema
        │   │   └── index.ts
        │   ├── App.tsx              # Componente principal do Dashboard
        │   └── index.tsx
        ├── package.json
        ├── tailwind.config.js       # Configuração de estilos com TailwindCSS
        └── tsconfig.json

```

## Pré-requisitos

| Ferramenta                   | Versão mínima | Para que serve                                                                                                   |
| ---------------------------- | ------------- | ---------------------------------------------------------------------------------------------------------------- |
| Node.js                      | A definir     | Executar o servidor Backend e ferramentas de build do Dashboard Frontend                                         |
| npm                          | A definir     | Gerenciamento das bibliotecas e dependências TypeScript, React.js e Express.js                                   |
| PlatformIO                  | A definir     | Ambiente de desenvolvimento para compilar e enviar o firmware escrito em C/C++                                   |
| Framework Arduino ESP32 Core | A definir     | Fornecer as bibliotecas base para interagir com o hardware da placa ESP32 DevKit V1                              |
| PostgreSQL                   | A definir     | Banco de dados relacional adotado para persistir sessões de navegação, telemetria e logs                         |
| Docker + Compose             | A definir     | Criação de contêineres locais para facilitar a execução do PostgreSQL durante o desenvolvimento                  |
| Prisma CLI                   | A definir     | Ferramenta de ORM (Object-Relational Mapping) para criar as tabelas e gerenciar as migrações do banco relacional |

## Setup inicial

### 1) Clonar e instalar dependências

```bash
git clone <repo-url>
cd <pasta-do-repositorio>
npm install
```
