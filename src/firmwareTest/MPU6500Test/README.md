# MPU6500Test — teste estático e dinâmico do yaw

Este projeto PlatformIO reutiliza o driver `src/firmware/src/hal/imu_mpu6500.cpp` do firmware principal. Ele não altera nem grava parâmetros de navegação permanentes.

## O que verifica

1. Comunicação I²C no endereço configurado (tenta também o endereço alternativo `0x68`/`0x69`).
2. Identificação `WHO_AM_I` e inicialização do MPU.
3. Calibração do giroscópio Z com o robô imóvel.
4. Leituras estáticas do acelerômetro, do giroscópio Z e da magnitude da aceleração.
5. Teste de orientação estática dos seis sentidos dos eixos do acelerômetro: +X, -X, +Y, -Y, +Z e -Z.
6. Teste dinâmico guiado: gira aproximadamente 90° para a esquerda (anti-horário visto de cima), registra o yaw, retorna para a orientação inicial girando para a direita e calcula o erro de fechamento.

## Como executar

Na pasta `src/firmwareTest/MPU6500Test/`:

```bash
pio run
pio run --target upload --upload-port /dev/ttyUSB0
pio device monitor --port /dev/ttyUSB0 --baud 115200
```

## Procedimento físico

- Mantenha os motores desabilitados. Faça a rotação manualmente; o teste não aciona motores.
- Fixe a IMU no chassi e mantenha o módulo nivelado durante os movimentos.
- Na pergunta inicial, digite `s` somente se o robô estiver parado, nivelado e com os motores desabilitados.
- Na etapa de giro para a esquerda, pressione Enter para começar a captura; então gire aproximadamente 90° no sentido anti-horário quando visto de cima e pressione Enter para encerrar a captura.
- Na etapa de retorno, pressione Enter para começar a captura; gire para a direita até a orientação original e pressione Enter novamente.
- No teste de eixos, siga as seis poses solicitadas. Em cada pose, o eixo indicado deve apontar verticalmente para cima; deixe o módulo imóvel e pressione Enter para coletar 20 amostras. Para cima em +Z, por exemplo, o vetor esperado do acelerômetro é aproximadamente (0, 0, +1) g.
- O teste de eixos verifica o ACELERÔMETRO, não o giroscópio: o giroscópio mede velocidade angular e não informa sozinho a orientação estática em relação à gravidade.
- Cada captura dinâmica de yaw tem limite de 15 segundos. A saída serial mostra yaw e velocidade angular Z durante a rotação.

Use o monitor serial a 115200 baud. O teste é interativo e exige duas pressões de Enter por segmento: uma para iniciar a captura e outra para encerrá-la.

## Como interpretar

- Para a montagem convencional, com o eixo +Z da IMU apontando para cima, um giro anti-horário deve produzir yaw positivo. Se o sinal for negativo, confira a orientação física do módulo e a configuração `imuYawSign` do `MotionController` (atualmente inicializada como `+1.0f`).
- A magnitude do primeiro giro é considerada próxima de 90° entre 60° e 120° (tolerância de ±30°).
- O retorno é considerado próximo da referência se o erro de fechamento for de até 20°.
- O sinal do movimento de retorno deve ser oposto ao sinal da ida.
- No teste de orientação dos eixos, uma pose passa quando o componente esperado tem pelo menos 0,70 g no sinal correto, os componentes transversais ficam até 0,35 g em magnitude e a magnitude total fica entre 0,70 e 1,30 g. As seis poses não são uma calibração metrológica.
- `PASS`/`WARN` do teste dinâmico avaliam uma manobra manual e não são uma calibração metrológica. Um resultado depende da velocidade do movimento, de quando Enter é pressionado, de a IMU permanecer nivelada e da orientação dos eixos do módulo.
- O yaw é obtido pela integração do giroscópio Z: é orientação relativa, não um norte absoluto, e pode acumular erro com o tempo.

O projeto não altera o driver compartilhado nem os demais testes individuais.
