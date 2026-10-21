# _Firmware_
Esta pasta deverá armazenar arquivos referentes a:

- Código-fonte para microcontroladores e SOCs: [Arduino](https://www.arduino.cc/), [ESP32](https://www.espressif.com/), [Raspberry Pi](https://www.raspberrypi.com/), dentre outros.
- Projetos de organização destes códigos, caso utilize IDEs: [Arduino IDE](https://www.arduino.cc/en/software/), [Visual Studio](https://visualstudio.microsoft.com/pt-br/downloads/) etc.
- Arquivos de configuração: ```CMakeLists.txt``` se for usar [CMake](https://cmake.org/download/), ```Makefile``` se for usar [make](https://www.cs.colby.edu/maxwell/courses/tutorials/maketutor/) etc.

Evite incluir:
- Artefatos de compilação: os arquivos de saída compilados (binários, arquivos objeto, .hex, .elf) devem ser gerados pelo processo de compilação e não incluídos no repositório.
- Arquivos temporários/específicos do sistema operacional: Arquivos como .DS_Store (macOS) ou arquivos de configuração do editor (a menos que esteja usando uma configuração compartilhada como .editorconfig).
- Configurações pessoais: Arquivos de configuração locais específicos da máquina do desenvolvedor.

> [!WARNING]
> **Não acrescente arquivos referentes a _hardware_ nesta pasta.** Eles deverão ser armazenados na pasta [hw](https://github.com/fcte-pi1/template/tree/main/hw) deste repositório.


## Integração de navegação, telemetria e histórico (HU-00/01/02/03/05/06/09/10/11/12)

O `main.cpp` compõe a percepção de paredes, estimativa de pose, mapa, missão Flood Fill, controlador de movimento, monitor de bateria, `SessionCoordinator` e `TelemetryClient`. Os componentes de navegação/controladoria não escrevem diretamente no WebSocket: o transporte usa uma fila FreeRTOS; a tarefa de comunicação gerencia Wi-Fi, reconexão e persistência/retransmissão dos dados relevantes. Mensagens periódicas de telemetria antigas podem ser descartadas para não acumular atraso.

### Configuração local

1. Copie `include/config/network_config.local.example.hpp` para `include/config/network_config.local.hpp`. Esse arquivo local está incluído no `.gitignore`; não versione credenciais.
2. Configure `MM_WIFI_SSID` e `MM_WIFI_PASSWORD`.
3. Configure `MM_TELEMETRY_HOST` com o endereço IP da máquina na rede local que executa o backend; não use `localhost` ou `127.0.0.1`, pois isso apontaria para o próprio ESP32. `MM_TELEMETRY_PORT` usa `3000` e `MM_TELEMETRY_PATH` deve permanecer `/ws?role=device`. Libere a porta no firewall para a rede local conforme as regras do laboratório.
4. `MM_MAZE_WIDTH` seleciona 4, 8 ou 12 colunas; a altura é 4. `MM_START_CORNER` usa 0=superior esquerdo, 1=superior direito, 2=inferior esquerdo, 3=inferior direito; `MM_INITIAL_HEADING` usa 0=Norte, 1=Leste, 2=Sul, 3=Oeste. Confira que esses valores correspondem à orientação física ao posicionar o robô.
5. Ajuste `MM_INA226_SHUNT_RESISTANCE_OHMS` e `MM_INA226_MAX_EXPECTED_CURRENT_A` conforme o módulo instalado. Se o INA226 não for detectado, tensão/corrente são enviadas como `null`, não como zero.

### Contrato WebSocket

O firmware conecta como `role=device`. O `session_start` é enviado antes de logs e telemetria para que o backend associe os registros a uma sessão. A finalização inclui o percurso em ordem; se não houver rede, ela permanece pendente em RAM e o `SessionBuffer` também tenta gravar o percurso fechado no LittleFS. O mapa mantém em cache a configuração e as últimas observações de cada célula, que são retransmitidas após reconexão. O formato de parede é triestado: `true`=parede, `false`=passagem medida e `null`=lado ainda desconhecido.

O controle de movimento e os sensores físicos precisam ser calibrados no robô real antes da demonstração. Os parâmetros do INA226 são hardware-dependentes; não interprete consumo/bateria como válido sem validar a calibração.
