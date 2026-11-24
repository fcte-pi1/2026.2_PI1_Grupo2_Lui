# Firmware Micromouse — ESP32 e PlatformIO

Exploração de labirintos 4×4, 8×4 e 12×4 com Flood Fill, mapa de paredes, encoders e IMU. O objetivo é o canto oposto ao início. **Ligar ou gravar a placa não inicia movimento.**

## Compilar e gravar

Abra esta pasta (`src/firmware`) no VS Code com PlatformIO IDE. O ambiente padrão é `esp32dev`.

```bash
pio run -e esp32dev
```

Depois de conferir a alimentação dos HC-SR04 e a adaptação dos sinais ECHO para a ESP32:

```bash
pio device list
pio run -e esp32dev -t upload --upload-port /dev/cu.usbserial-0001
pio device monitor -e esp32dev -p /dev/cu.usbserial-0001 -b 115200
```

Confira o nome da porta na lista; somente um programa pode utilizá-la por vez. No Mac, se `pio` não estiver no PATH, use o terminal integrado do PlatformIO ou `~/.platformio/penv/bin/pio`.

## Operar pelo monitor

Envie uma letra por comando:

| Comando | Função |
|---|---|
| `h` | Ajuda |
| `s` / `d` | Último quadro / ecos brutos; conferir idade dos dados |
| `i` | Diagnóstico parado; termina com sensores pausados |
| `r` | Retomar somente sensores |
| `p` | Parar e pausar sensores |
| `x` | Interromper ou cancelar a tentativa |
| `g` | Preparar missão completa; partir após espera de 5 segundos |
| `t` / `c` | Testes guiados de movimento; não são a missão completa |

**A espera de 5 segundos só existe depois de gravar esta versão.** Compilar não atualiza a ESP32. Há uma tentativa por reinicialização, com limite de 180 segundos após a espera. `Ctrl+C` fecha o monitor e não para o robô: envie `x` antes quando necessário.

Padrão: 4×4, início (0,0), frente Leste, destino (3,3), passo de 180 mm (170 mm livres + parede de 10 mm). `src/config/run_config.hpp` utiliza `MM_MAZE_WIDTH`, `MM_START_CORNER` e `MM_INITIAL_HEADING` da configuração local ou seus padrões.

[Guia completo para operar sozinho, com comandos e configuração](OPERACAO.md).

## Verificação e limites

```bash
pio test -e motion_native
pio test -e native
```

Os testes incluem 768 percursos completos gerados, becos, ciclos, objetivo inacessível e falhas de leitura. Uma falha sem eco no hardware real é **desconhecida** e não autoriza avançar. Retaguarda desconhecida exige giro e nova observação; recuo automático só usa aresta conhecida como aberta.

O HC-SR04 padrão especifica 5 V. Antes de mudar a alimentação, confira a adaptação do ECHO para GPIO de 3,3 V. Os offsets continuam provisórios. Compilação e simulação não garantem precisão física, espaço para girar ou conclusão de qualquer labirinto em 180 segundos. Valide as leituras, avanço de 18 cm e curvas antes da missão completa.

O benchmark de PositionTracker mantém o orçamento absoluto de 500 µs. A comparação percentual instável entre operações de trabalho diferente foi removida; o teste no computador não mede o prazo real do controle na ESP32.

## Organização

| Arquivo/pasta | Responsabilidade |
|---|---|
| `src/main.cpp` | Inicialização, comandos e coordenação |
| `src/config/` | Geometria, pinos, calibração e operação |
| `src/hal/` | Sensores, motores e armazenamento |
| `src/navigation/` | Mapa, percepção, rastreamento e Flood Fill |
| `src/control/` | Movimento, filtro lateral e partida cancelável |
| `test/` | Testes nos ambientes nativos, sem placa ou motores |
| `include/` | Headers de compatibilidade; implementações em `src/` |

`firmware.ino` permanece por compatibilidade histórica. O fluxo verificado nesta versão é PlatformIO; o guia antigo de Arduino IDE e partida automática foi substituído.
