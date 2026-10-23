# Testes isolados de hardware — ESP32 MicroMouse

Esta pasta contém pequenos projetos PlatformIO independentes para verificar os periféricos ligados ao ESP32 sem alterar o firmware principal em `src/firmware/`. Cada teste imprime logs com timestamp, condição do ESP32, GPIOs/barramento utilizados, resultado de pré-verificação de conexão e o resultado do diagnóstico.

## Princípio obrigatório de execução

Cada projeto segue a ordem **inicialização segura → teste de conexão → diagnóstico funcional**. Se a conexão não for confirmada, as etapas funcionais daquele componente são bloqueadas. Componentes sem retorno eletrônico de presença (DRV8833, US-100 e encoders) exigem uma evidência física/operacional descrita nos respectivos testes; não é possível confirmar apenas por software um fio rompido ou alimentação ausente em todos esses casos.

## Componentes incluídos

| Pasta | Componente e abrangência | Interface/pinos do projeto |
|---|---|---|
| `US100Test/` | US-100 frontal, esquerdo e direito | TRIG/ECHO: GPIO 4/35, 14/36, 27/39 |
| `DRV8833Test/` | Ponte H DRV8833 e motores N20 esquerdo/direito | IN1/IN2: 16/17 e 18/19; nSLEEP: GPIO 5; PWM 20 kHz |
| `EncoderN20Test/` | Encoders dos dois motores N20 | Esquerdo A/B: GPIO 32/33; direito A/B: GPIO 25/26 |
| `MPU6500Test/` | IMU MPU6500 (aceita identificação compatível MPU6050, como faz o driver do projeto) | I²C SDA/SCL: GPIO 21/22; endereços 0x68 ou 0x69 |
| `INA226Test/` | Medidor de tensão/corrente INA226 | I²C SDA/SCL: GPIO 21/22; endereço padrão 0x40 |

O regulador MP1584 não possui sinal de diagnóstico conectado à ESP32 neste esquema; valide sua tensão de saída com multímetro conforme a documentação elétrica. O teste do DRV8833 não lê um pino de falha, pois o projeto não possui um sinal de feedback desse tipo ligado à ESP32: a confirmação de movimento é manual e explicitamente registrada.

## Como executar

1. Descompacte a pasta `src/firmwareTest/` na raiz do repositório, preservando o caminho `src/firmwareTest/`.
2. Instale/abra o PlatformIO e conecte a ESP32 DevKit V1. Grave **um projeto de teste por vez**; gravar um teste substitui temporariamente o firmware de navegação até que ele seja gravado novamente.
3. No terminal, navegue para a pasta do teste e execute, por exemplo:

   ```bash
   cd src/firmwareTest/MPU6500Test
   pio run
   pio run --target upload --upload-port /dev/ttyUSB0
   pio device monitor --port /dev/ttyUSB0 --baud 115200
   ```

   Ajuste a porta serial conforme o sistema (`/dev/ttyUSB0`, `/dev/ttyACM0` ou `COMx`). No VS Code, também é possível abrir diretamente a pasta do teste como projeto PlatformIO e usar **Build**, **Upload** e **Monitor**.
4. Siga as instruções interativas do monitor serial. Digite ENTER ou o comando informado no próprio log. Os logs usam 115200 baud.
5. Depois do diagnóstico, interrompa o monitor, abra `src/firmware/` como projeto PlatformIO e faça upload do firmware principal para voltar à operação normal.

## Segurança e interpretação

- Confirme a pinagem no esquemático vigente; `include/config/board_pins.hpp` registra a pinagem atualmente utilizada pelo firmware.
- Durante o teste do DRV8833, eleve/prenda o chassi para que as rodas girem livremente e mantenha mãos, cabos e objetos afastados. O acionamento só ocorre após confirmação explícita pelo monitor serial e usa PWM baixo.
- Durante a calibração da MPU6500, mantenha o robô imóvel, nivelado e com os motores desabilitados.
- Durante a pré-verificação dos US-100, coloque um objeto plano em frente ao sensor indicado, preferencialmente entre 10 e 50 cm. Ausência de eco pode indicar alvo fora de alcance, diafonia, modo/pinagem incorretos ou falta de alimentação; o software não distingue todas essas causas sozinho.
- Os testes de INA226 usam, inicialmente, os mesmos parâmetros-exemplo do firmware (`0.01 Ω` de shunt e `5 A`). **Confirme a resistência do shunt e a corrente máxima real do módulo** antes de interpretar corrente/potência. A calibração rejeita combinações acima da faixa documentada pelo driver.
- Estes são testes de bancada, não testes de autonomia do labirinto. Não atualizam o firmware principal nem a documentação existente.
