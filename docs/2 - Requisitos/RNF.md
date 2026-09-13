# Requisitos não Funcionais

Eles não tratam dos serviços diretos, mas descrevem as propriedades globais, as restrições e as qualidades estruturais que o projeto precisa respeitar. Eles determinam "como" a entrega vai operar, abordando aspectos como confiabilidade, desempenho, segurança e usabilidade.

---

## RNF Declarados

| **ID** | **Descrição** | **Prioridade** | **Responsáveis** | **Link Github Projects** |
|:------:|---------------|:--------------:|------------------|--------------------------|
| **RNF01** | Os dados da execução devem ser armazenados de forma persistente para consultas posteriores. | Must Have |  | - |
| **RNF02** | A bateria deve ter uma capacidade suficiente para realizar 3 trajetos ou 30 minutos de operação contínua. | Must Have |  | - |
| **RNF03** | Deve ter uma bateria recarregável. | Must Have |  | - |
| **RNF04** | O robô deve operar em corredores formados por células de 18 cm². | Must Have |  | - |
| **RNF05** |  O sistema de controle de motores deve garantir que a velocidade e a frenagem impeçam o robô de colidir com as paredes com força suficiente para causar danos físicos à estrutura do labirinto. | Must Have |  | - |
| **RNF06** |  Resistir a inversão de polaridade. | ShouldHave |  | - |
| **RNF07** |  Garantir que os sensores detectem as paredes com taxa de erro inferior a 30%. | CouldHave |  | - |
| **RNF08** |  Massa total do robô deve ser inferior a 500 g. | CouldHave |  | - |
| **RNF09** |  Exibir a telemetria no sistema web com latência mínima de até X (Definir valor). | WontHave |  | - |