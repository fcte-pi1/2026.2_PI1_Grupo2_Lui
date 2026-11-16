# Bancada guiada

Projeto independente do firmware de missão; usa os drivers existentes por arquivos de inclusão .cpp separados.
Não inicia navegação nem movimento ao ligar. EEP/nSLEEP pressupõe GPIO5 conforme docs/7.3.
Compilar: `pio run -e esp32dev`. Monitor: 115200 baud.

Comandos: h ajuda, i IMU imóvel, s distâncias, c encoders, z zerar encoders, x parar/desarmar.
Somente com rodas suspensas: a arma por 10 segundos; l/r aplica um pulso único de 200 ms a PWM 230/255
na roda esquerda/direita. Novo pulso exige novo armamento. O pulso é limitado por tempo mesmo sem
novo comando USB. Não há avanço ou curva automáticos neste primeiro diagnóstico.

Começar com bateria desligada para upload. Alimentar periféricos apenas após conferir a montagem.
A leitura s marca ausência de eco como INVALIDO; não interpreta como corredor livre.

Saida de motores usa LEDC direto a 20 kHz/8 bits e espera 10 ms apos nSLEEP HIGH, conforme o sketch validado pelo usuario. O pulso permanece limitado a 200 ms.

## Giro no piso

Em área livre de 30 x 30 cm, com ponto médio do eixo centralizado: i calibra a IMU com robô imóvel; b arma por 10 segundos; t inicia um giro relativo de 30 graus à direita. x interrompe/desarma. A versão V2 usa controle de velocidade independente por roda e sincronização por progresso em milímetros, com frenagem individual quando a diferença passa de 5 mm. Há limite de tempo de 3 segundos, verificação de sentido, de progresso e de deslocamento dos encoders. A frenagem final aguarda repouso. A calibração por vídeo é provisória; validação de movimento e da precisão do giro permanece experimental. Não inicia movimento ao ligar.

Comandos atuais de giro: t=30 graus à direita; q=90 graus à direita; e=90 graus à esquerda. Todos exigem calibração i e armamento b, têm limites proporcionais ao alvo e controle independente por roda. O sinal do yaw e dos encoders é normalizado conforme o sentido solicitado.

Comando u: giro único de 180 graus à direita, com limite de 11 segundos e yaw acumulado continuamente ao cruzar ±180 graus. Validar apenas em área livre de 30 x 30 cm. Testes de continuidade angular: pio test -e native.

Avanço no piso: f comanda 30 mm após calibração i e armamento b. Exige leitura frontal válida de pelo menos 8 cm antes de começar, freia se a leitura cair para 5 cm ou ficar inválida, consulta encoders/IMU, corrige velocidade por roda e direção, limita cada roda a 50 mm, desvio de yaw a 15 graus e duração a 4 s. O diagnóstico para e aguarda novo comando; não há navegação automática. Conferir o deslocamento com régua, pois a calibração dos encoders ainda é provisória.
