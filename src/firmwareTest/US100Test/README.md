# US100Test — teste de bancada para HC-SR04

> O diretório `US100Test` foi mantido para preservar o caminho esperado pelo repositório. O teste está configurado para **HC-SR04**, não para US-100.

O HC-SR04 usa o mesmo protocolo de disparo/retorno `TRIG`/`ECHO` que o driver ultrassônico já implementa, portanto `src/driver.cpp` reutiliza `src/firmware/src/hal/ultrasonic_us100.cpp` sem duplicar o driver. A faixa configurada no teste é de 2 a 400 cm.

## Pinagem conferida na documentação do repositório

| Sensor | TRIG do HC-SR04 | ECHO do HC-SR04 (passa por divisor) | Pino na placa |
|---|---:|---:|---|
| Frontal | GPIO 4 | GPIO 35 | D4 / D35 |
| Esquerdo | GPIO 14 | GPIO 36 | D14 / VP (GPIO36) |
| Direito | GPIO 27 | GPIO 39 | D27 / VN (GPIO39) |

A pinagem corresponde à seção 2.1 de `docs/4.3 - Projeto conceitual de hardware.md` e ao esquemático `SCH_Ligacao_dos_componentes_1-P1_2026-09-25.png`. GPIO35, GPIO36 e GPIO39 são entradas, apropriadas para `ECHO`.

## Atenção elétrica obrigatória

- Alimente o HC-SR04 em **5 V** (VCC) e conecte seu GND ao GND da ESP32.
- O sinal `TRIG` de 3,3 V da ESP32 pode ser ligado ao `TRIG` do módulo.
- **Não ligue ECHO diretamente à ESP32.** O HC-SR04 pode entregar aproximadamente 5 V em ECHO, enquanto o ESP32 aceita no máximo 3,6 V de acordo com os limites absolutos do datasheet.
- Use um divisor de tensão por sensor, por exemplo: `ECHO do HC-SR04 -> resistor de 1 kΩ -> nó do GPIO`; ligue `resistor de 2 kΩ` desse nó ao GND. Ligue o GPIO ao nó entre os dois resistores. Isso reduz aproximadamente 5 V para 3,3 V. Um conversor de nível lógico apropriado também serve.
- Desligue a alimentação antes de alterar os fios. Confirme com multímetro o 5 V e a continuidade do GND comum.

## Como executar

Execute no terminal, dentro desta pasta:

```bash
pio run
pio run --target upload --upload-port /dev/ttyUSB0
pio device monitor --port /dev/ttyUSB0 --baud 115200
```

Ajuste `/dev/ttyUSB0` se a porta for diferente. Para ver o pré-check, posicione um alvo plano entre 10 e 50 cm diante do sensor indicado e pressione ENTER quando solicitado.

## Como o pré-check decide

Cada sensor recebe cinco tentativas, espaçadas em pelo menos 60 ms. O teste exige ao menos três ecos válidos e consistentes com a mediana das leituras antes de liberar a fase funcional. Uma inicialização de GPIO não conta como confirmação de conexão. Se o pré-check falhar, o diagnóstico funcional daquele sensor é ignorado.

A ausência de eco, isoladamente, não prova que o sensor está desconectado. Verifique, em especial, alimentação de 5 V, GND comum, TRIG/ECHO corretos, divisor no ECHO, alvo dentro da faixa e qualidade dos jumpers.

**Atenção:** gravar este teste substitui o firmware que estiver instalado na ESP32. Ao terminar a bancada, grave novamente o firmware principal a partir de `src/firmware/`.
