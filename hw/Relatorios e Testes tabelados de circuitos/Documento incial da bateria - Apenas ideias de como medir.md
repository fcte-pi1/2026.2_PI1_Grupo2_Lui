# VERIFICANDO A BATERIA DO MICRO MAUSE

Neste documento, será explicado como calcular a bateria do rato.

## DIVISOR DE TENSÃO

Para ler uma bateria de 7,5V (que quando totalmente carregada pode chegar perto de 8,4V se for de Lítio 2S) na entrada D34 do ESP32, o método padrão e mais seguro é usar um Divisor de Tensão com dois resistores $R_1$ e $R_2$.

O ESP32 tolera no máximo 3,3V em seus pinos. Vamos projetar o divisor para garantir que a leitura fique segura:

|Componente / Parâmetro|Valor Recomendado|Função no Circuito|
|---|---|---|
|Resistor de Entrada $(R_{1})$ |	$10kΩ$ |	Conectado entre o Positivo da Bateria e o pino D34.|
|Resistor de Chão $(R_{2})$	| $(4,7k\Omega)$ |	Conectado entre o pino D34 e o GND (Negativo).|
|Tensão Máxima Suportada |	$(10,3V)$ |	Margem de segurança ideal para a sua bateria de 7,5V.|
|Resolução do ADC do ESP32 |	$(4095)$ (12 bits)	| O ESP32 usa por padrão 12 bits, gerando valores de 0 a 4095.|


## COMO CALCULAR A TENSÃO NO CÓDIGO USANDO O DIVISOR DE TENSÃO:

Com os resistores sugeridos $(10k\Omega)$ e $(4,7k\Omega)$, a fórmula matemática para colocar no código do seu ESP32 será:

$Tensão da Bateria = (\frac{M}{4095})*3,3*(\frac{10+4,7}{4,37})$

A explicação detalhada de cada termo presente na conta:

- \(M\)	O valor bruto (número inteiro) lido pelo comando analogRead(34) no seu código.
- \(4095\)	A resolução máxima do conversor do ESP32 (por padrão, ele trabalha em 12 bits, gerando números de 0 a 4095).
- $\frac{M}{4095}$	A proporção da leitura. Transforma o número bruto em uma porcentagem (de 0% a 100%) da escala do chip.
- \(3,3\)	A tensão máxima de referência que o pino do ESP32 aceita receber (3,3 Volts).

|![simulaçao da bateria](./Simulações_de_circuitos_eletrônicos/export.png)|

Onde o medidor2.V corresponde a nossa tensão lida pela esp32 como valor bruto.

## O valor bruto (número inteiro) lido pelo comando analogRead(34) no código

entrada d34 da esp32

A resolução máxima do conversor analógico-digital (ADC) do ESP32 DevKit V1 (30 pinos) é de 12 bits. 
Detalhes do conversor (ADC):

	-Resolução: 12 bits, o que gera valores inteiros de 0 a 4095 (2¹² - 1).
	-Canais: Possui 15 canais analógicos disponíveis na versão de 30 pinos, divididos entre o ADC1 e o ADC2.
	-Tensão de entrada: Geralmente de 0 V a 3,3 V (ou até ~3,9 V dependendo da atenuação configurada no software).
	-Conversor Digital-Analógico (DAC): Se você se referir ao conversor inverso (DAC - digital para analógico), ele possui 8 bits de resolução (valores de 0 a 255) em 2 canais (pinos GPIO 25 e 26). [1, 2, 3, 4, 5]

Se entrarem 2V no pino D34 (que faz parte do ADC1), o valor retornado depende da atenuação configurada no seu código.
Por padrão, a maioria das plataformas (como a Arduino IDE e o MicroPython) configura o ESP32 com a atenuação máxima de 11 dB para permitir leituras de até 3,3V.

### O cálculo (para a configuração padrão de 11 dB):

Se a tensão máxima (3,3V) equivale ao valor máximo de 12 bits (4095), a regra de três é:

$Valor =\ \frac{2\text{V}\ \times\ 4095}{3,3\text{V}}\ \approx\ 2481,8$

Nota: O ADC do ESP32 é conhecido por não ser perfeitamente linear (especialmente perto de 0V e acima de 3,2V), por isso o valor real lido na prática pode oscilar ligeiramente perto de 2482.

## No codego

O  valor bruto (raw value) que a função analogRead(34) vai salvar na sua variável dentro da Arduino IDE.
Por ser um número inteiro, o código processa a leitura da seguinte forma:

```
int valorBruto = analogRead(34); // Se a tensão for 2V, valorBruto será aproximadamente 2482
```

Se o seu objetivo for converter esse número de volta para a tensão real (em Volts) dentro do código para exibir em um display ou no Monitor Serial, você deve usar a seguinte fórmula matemática:

```
float tensao = (valorBruto * 3.3) / 4095.0; // Vai resultar em aproximadamente 2.0V
```

Para implementar essa equação na Arduino IDE, você deve usar variáveis do tipo float para não perder as casas decimais nos cálculos da divisão.
Aqui está o código direto:

```

// Definição do pino de leitura analógica
const int pinoADC = 34;

void setup() {
  // Inicializa a comunicação serial para ver o resultado
  Serial.begin(115200);
}

void loop() {
  // 1. Lê o valor bruto (M) do ADC (0 a 4095)
  int M = analogRead(pinoADC);
  
  // 2. Aplica a fórmula exata da imagem
  // Nota: Usamos .0 nos números para garantir o cálculo em ponto flutuante (float)
  float tensaoBateria = (M / 4095.0) * 3.3 * ((10.0 + 4.7) / 4.7);
  
  // 3. Exibe o resultado no Monitor Serial
  Serial.print("Valor Bruto (M): ");
  Serial.print(M);
  Serial.print(" | Tensão da Bateria: ");
  Serial.print(tensaoBateria);
  Serial.println(" V");
  
  // Aguarda 1 segundo antes da próxima leitura
  delay(1000);
}

```
## DICA

Como os valores dos resistores ($10\text{ k}\Omega$ e $4,7\text{ k}\Omega$) e as tensões máximas são fixos, a parte final da sua fórmula

 $\left(3.3 \cdot \frac{14,7}{4,7}\right)$ sempre resultará em aproximadamente $10,3212$.

Se quiser deixar o código ligeiramente mais rápido para o processador do ESP32, você pode substituir por:

```
float tensaoBateria = (analogRead(34) / 4095.0) * 10.3212;
```