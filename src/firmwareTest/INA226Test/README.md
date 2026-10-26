# Teste de hardware — INA226

Este projeto PlatformIO testa o INA226 em quatro condições descritas em `docs/7.3 - Testes de hardware/7.3 - Testes de hardware.md`: repouso, motores livres a 50% PWM, motores livres a 100% PWM e carga máxima/manobra.

## Configuração padrão

- ESP32 DevKit V1; SDA GPIO 21, SCL GPIO 22.
- Endereço I²C padrão `0x40`, clock de teste de 400 kHz.
- Alimentação lógica de 3,3 V.
- Shunt padrão deste teste: `0.1 Ω`, conforme a seção 7.3.
- Corrente máxima de calibração padrão: `0.8 A`, para respeitar a faixa de shunt de aproximadamente ±81,92 mV do INA226.

**Atenção: há uma incompatibilidade na especificação publicada.** Um shunt de `0.1 Ω` com corrente de `850 mA` produz `85 mV`, acima da faixa de ±81,92 mV do INA226. Portanto, a corrente da condição de carga máxima não pode ser validada com garantia usando esses dois parâmetros. O programa emite um aviso e classifica a comparação de corrente como inconclusiva quando a corrente de referência ultrapassa a faixa calibrada. Confirme o valor do resistor instalado no módulo antes do ensaio.

O firmware principal atualmente usa `0.01 Ω` e `5 A` como parâmetros de calibração. Se a resistência REAL do shunt for `0.01 Ω`, configure este teste para os mesmos valores antes de gravá-lo. Os parâmetros podem ser alterados em **Project Configuration / Build Flags** ou diretamente pelas definições no topo de `src/main.cpp`:

```cpp
#define INA226_TEST_SHUNT_RESISTANCE_OHMS 0.01f
#define INA226_TEST_MAX_EXPECTED_CURRENT_A 5.0f
```

Só use valores confirmados pela identificação do módulo ou por medição; a calibração depende diretamente da resistência do shunt.

## Execução

Na pasta `src/firmwareTest/INA226Test/`, execute:

```bash
pio run
pio run --target upload --upload-port /dev/ttyUSB0
pio device monitor --port /dev/ttyUSB0 --baud 115200
```

Substitua a porta serial conforme o sistema operacional. O teste começa verificando ACK I²C, registradores de identificação do INA226 e registro de calibração. Se essa verificação falhar, as etapas funcionais não são executadas.

Para cada condição:

1. Ajuste fisicamente a carga indicada no monitor serial; aguarde a leitura estabilizar e pressione ENTER. Digite `s` para pular a condição.
2. O teste registra 10 amostras de tensão de barramento, tensão de shunt, corrente e potência e calcula as médias.
3. Digite a tensão e a corrente medidas pelo multímetro **nessa execução**. O programa calcula o erro relativo assinado de tensão e corrente, imprime o resultado por condição e apresenta um resumo final.

Os valores do documento (7,82 V/62,0 mA, 7,65 V/185,0 mA, 7,48 V/320,0 mA e 7,20 V/850,0 mA) são exibidos apenas como referência histórica. Eles não devem substituir as medições atuais do multímetro.

## Critérios registrados no teste

- Erro relativo por condição para tensão e corrente: menor que `0,27%`, conforme a matriz de testes (ID 3.8).
- Erro relativo absoluto médio de tensão: menor que `0,25%`, conforme a descrição do ensaio INA226.
- Se a tensão de shunt chegar a 80 mV, se a corrente de referência exceder a corrente máxima calibrada ou se faltar amostra, a comparação de corrente ou a etapa é considerada inconclusiva/incompleta, e não aprovada automaticamente.

Há uma segunda divergência nos resultados documentados: a última linha registra erro de tensão de `-0,28%`, acima do limite por ponto de `<0,27%`; os erros de corrente calculados a partir dos valores publicados também superam `0,27%` em algumas condições. O teste usa os limites explícitos para que a execução revele essas divergências em vez de reproduzir um status de aprovação fixo.
