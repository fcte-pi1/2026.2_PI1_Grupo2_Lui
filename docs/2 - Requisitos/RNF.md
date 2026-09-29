# Requisitos Não Funcionais

Requisitos não funcionais não possuem em seu escopo o tratamento dos serviços diretos, mas descrevem as propriedades globais, as restrições e as qualidades estruturais que o projeto precisa respeitar. Eles determinam "como" a entrega vai operar, abordando aspectos como confiabilidade, desempenho, segurança e usabilidade.

---

## RNF Declarados

| **ID** | **Nome do Requisito** | **Descrição** | **Prioridade** | **Responsáveis** | **Link Github Projects** |
|:------:|-----------------------|---------------|:--------------:|------------------|--------------------------|
| **RNF01** | Persistência de Dados Operacionais | Armazenar os dados da execução de forma persistente para consultas posteriores. | Must Have | Software | [#33](https://github.com/fcte-pi1/2026.2_PI1_Grupo2_Lui/issues/33) |
| **RNF02** | Autonomia da Bateria | Garantir que a bateria tenha capacidade suficiente para realizar 3 trajetos ou até 30 minutos de operação contínua. | Must Have | Energia | [#34](https://github.com/fcte-pi1/2026.2_PI1_Grupo2_Lui/issues/34) |
| **RNF03** | Fonte de Energia Recarregável | Possuir bateria recarregável e acessivel sem desmontar todo o robo. | Must Have | Energia | [#35](https://github.com/fcte-pi1/2026.2_PI1_Grupo2_Lui/issues/35) |
| **RNF04** | Compatibilidade de Dimensão Mecânica | Operar o robô em corredores formados por células de 18 cm². | Must Have | Estruturas | [#36](https://github.com/fcte-pi1/2026.2_PI1_Grupo2_Lui/issues/36) |
| **RNF05** | Tolerância a Colisão e Controle Impacto | Garantir por meio do sistema de controle de motores que a velocidade e a frenagem impeçam o robô de colidir com as paredes com força suficiente para causar danos físicos à estrutura do labirinto. | Must Have | Software | [#37](https://github.com/fcte-pi1/2026.2_PI1_Grupo2_Lui/issues/37) |
| **RNF06** | Integridade Estrutural | Assegurar que o robô se mantenha intacto no final do trajeto. | Must Have | Estruturas | [#38](https://github.com/fcte-pi1/2026.2_PI1_Grupo2_Lui/issues/38) |
| **RNF07** | Autonomia de Navegação Embarcada | Operar de forma 100% autônoma, sem necessidade de interferência humana externa durante o percurso. | Must Have | Software | [#39](https://github.com/fcte-pi1/2026.2_PI1_Grupo2_Lui/issues/39) |
| **RNF08** | Proteção contra Inversão de Polaridade | Resistir à inversão de polaridade. | Should Have | Eletrônica | [#40](https://github.com/fcte-pi1/2026.2_PI1_Grupo2_Lui/issues/40) |
| **RNF09** | Precisão de Leitura dos Sensores | Garantir que os sensores detectem as paredes com taxa de erro inferior a 30%. | Could Have | Eletrônica | [#41](https://github.com/fcte-pi1/2026.2_PI1_Grupo2_Lui/issues/41) |
| **RNF10** | Limite de Massa Total | Ponderar massa total do robô para ser inferior a 500 g. | Could Have | Estruturas | [#42](https://github.com/fcte-pi1/2026.2_PI1_Grupo2_Lui/issues/42) |
| **RNF11** | Latência na Exibição da Telemetria | Exibir a telemetria no sistema web com latência mínima de até 100ms. | Won't Have | Software | [#43](https://github.com/fcte-pi1/2026.2_PI1_Grupo2_Lui/issues/43) |
| **RNF12** | Dimensionamento da folga | Espaço entre envoltória do robô e paredes da célula do labirinto deve considerar tolerância de fabricação/montagem, evitando atrito ou emperramento. | Could Have | Estruturas | [#186](https://github.com/fcte-pi1/2026.2_PI1_Grupo2_Lui/issues/186)|
| **RNF13** | Resistencia à impactos | Material e junções do chassi devem suportar colisões leves recorrentes sem falha ou desalinhamento. | Should Have | Estruturas | [#187](https://github.com/fcte-pi1/2026.2_PI1_Grupo2_Lui/issues/187) |
| **RNF14** | Estabilização dinamica | Centro de gravidade e base de apoio (wheelbase/track width{distância entre as rodas}) dimensionados para evitar tombamento em deslocamento, giros e repouso. | Should have | Estruturas | [#188](https://github.com/fcte-pi1/2026.2_PI1_Grupo2_Lui/issues/188) |
| **RNF15** | Isolamento da bateria | O compartimento interno deve isolar fisicamente a bateria de qualquer contato condutor indesejado com a estrutura mecânica. | Should have | Estruturas | [#189](https://github.com/fcte-pi1/2026.2_PI1_Grupo2_Lui/issues/189) |
| **RNF16** | Organização interna de componentes | Fios, sensores e componentes não devem estar suscetíveis a interferências, não planejadas, entre si. | Could Have | Estruturas | [#190](https://github.com/fcte-pi1/2026.2_PI1_Grupo2_Lui/issues/190) |
| **RNF17** | Tracionar a roda | Rodas com material/perfil que maximizem aderência, minimizando derrapagem e erro odométrico. | Could have | Estruturas | [#191](https://github.com/fcte-pi1/2026.2_PI1_Grupo2_Lui/issues/191) |
| **RNF18** | Fixação da motriz | Chassi deve prover pontos de fixação rígidos para motores/atuadores, capazes de transmitir torque sem deformação | Wont Have | Estruturas | [#192](https://github.com/fcte-pi1/2026.2_PI1_Grupo2_Lui/issues/192) |
| **RNF19** | Suportar de sensoriamento | Pontos de fixação para sensores de parede, posicionados sem obstruir o campo de detecção e sem folga/vibração que degrade a leitura. | Could | Estruturas | [#193](https://github.com/fcte-pi1/2026.2_PI1_Grupo2_Lui/issues/193) |
| **RF20** | modificações na bateria | Bateria acessível e substituível entre tentativas, sem desmontagem completa do robô. | Should have | Estruturas | [#194](https://github.com/fcte-pi1/2026.2_PI1_Grupo2_Lui/issues/194) |