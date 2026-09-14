# Requisitos Não Funcionais

Requisitos não funcionais não possuem em seu escopo o tratamento dos serviços diretos, mas descrevem as propriedades globais, as restrições e as qualidades estruturais que o projeto precisa respeitar. Eles determinam "como" a entrega vai operar, abordando aspectos como confiabilidade, desempenho, segurança e usabilidade.

---

## RNF Declarados

| **ID** | **Descrição** | **Prioridade** | **Responsáveis** | **Link Github Projects** |
|:------:|---------------|:--------------:|------------------|--------------------------|
| **RNF01** | Armmazenar os dados da execução de forma persistente para consultas posteriores. | Must Have |  | - |
| **RNF02** | Garantir que a bateria tenha capacidade suficiente para realizar 3 trajetos ou 30 minutos de operação contínua. | Must Have |  | - |
| **RNF03** | Possuir bateria recarregável. | Must Have |  | - |
| **RNF04** | Operar o robô em corredores formados por células de 18 cm². | Must Have |  | - |
| **RNF05** | Garantir por meio do sistema de controle de motores que a velocidade e a frenagem impeçam o robô de colidir com as paredes com força suficiente para causar danos físicos à estrutura do labirinto. | Must Have |  | - |
| **RNF06** |   Assegurar que o robô se mantenha intacto no final do trajeto. | Must Have |  | - |
| **RNF07** |   Operar de forma 100% autônoma, sem necessidade de interferência humana externa durante o percurso. | Must Have |  | - |
| **RNF08** |  Resistir a inversão de polaridade. | Should Have |  | - |
| **RNF09** |  Garantir que os sensores detectem as paredes com taxa de erro inferior a 30%. | Could Have |  | - |
| **RNF10** |  Ponderar massa total do robô para ser inferior a 500 g. | Could Have |  | - |
| **RNF11** |  Exibir a telemetria no sistema web com latência mínima de até 100ms. | Won tHave |  | - |