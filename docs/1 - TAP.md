# Termo de Abertura do Projeto

> Termo de abertura do projeto / Project Charter. Um documento publicado pelo iniciador ou patrocinador do projeto que autoriza formalmente a existência de um projeto e fornece ao gerente do projeto a autoridade para aplicar os recursos organizacionais nas atividades do projeto.

## Visão Geral do Projeto

### Dados do projeto

* **Nome do Projeto:** Robo Labirinto
* **Data de Início:** 02/07
* **Data de Término:** 04/12
* **Patrocinador:** Universidade de Brasília

### Objetivos

- **Specific:** Desenvolver um robô autônomo (*micromouse*) capaz de mapear e resolver 3 labirintos de formatos desconhecidos **( 4x4, 8x4 e 12x4 )**, além de construir uma plataforma web para exibição da telemetria em tempo real (trajeto, bateria, velocidade, tempo) e persistência em banco de dados.
- **Measurable:**  
    - Resolver cada labirinto dentro do limite máximo de 10 minutos.
    - Obter a pontuação máxima de resolução (conclusão na 1ª tentativa para nota 10).
    - Exibir todos os dados de telemetria no sistema web para garantir o fator multiplicador 1,0 na avaliação.
    - Manter as dimensões físicas do robô estritamente abaixo do limite de **16,5cm x 16,5cm**.
- **Agreed:** Alinhado e acordado entre todos os integrantes da equipe multidisciplinar e os professores orientadores das turmas de Projeto Integrador 1 (FCTE/UnB).
- **Realistic:** Viável de ser executado com os conhecimentos integrados das engenharias da FCTE, contando com a construção prévia de uma pista de testes simplificada **4x4** para validação e ajuste do sistema antes das avaliações finais.
- **Time Bound:** Finalizar o projeto e apresentar a solução completa na avaliação final até a data limite de **04/12**, cumprindo todas as entregas intermediárias agendadas (AP2 a AP20).

### Público-Alvo

- **Professores e Avaliadores da FCTE/UnB:** Responsáveis por acompanhar o desenvolvimento, avaliar os entregáveis da disciplina e validar os testes de integração e a apresentação final.
- **Membros da Equipe de Engenharia:** Estudantes dos cursos de engenharia da FCTE que aplicarão conhecimentos de estruturas, energia, hardware e software.

### Descrição do Problema

A resolução autônoma de labirintos é um desafio clássico de engenharia que exige a integração multidisciplinar de engenharia de estruturas, energia, hardware e software. 

O robô deve ser projetado para navegar em células de **18cm** de lado, com paredes brancas de **5cm** de altura **1,2cm** de espessura (com topo vermelho) e chão preto. O veículo parte de um beco sem saída em um canto até a área objetivo no canto oposto. 

Existem restrições rígidas de projeto:

- Dimensões máximas de **16,5cm** de largura e comprimento (sem limite de altura).
- Proibição de voar, pular, escalar ou usar propulsão por combustão/foguetes.
- Proibição de alterar o código de computador ou a memória do robô durante a resolução do trajeto.
- Necessidade de transmitir dados de telemetria em tempo real para um sistema web próprio.

![Imagem de Labirinto](./figs/TAP/Labirinto.png)

### Indicadores

1. **Estudantes de Engenharia da FCTE/UnB:** Alunos matriculados na disciplina de Projeto Integrador 1.
2. **Equipes de Competitivos de Robótica (*Micromouse*) do DF:** Grupos interessados em plataformas de navegação autônoma e mapeamento.
3. **Laboratórios e Instituições de Ensino no DF:** Ambientes acadêmicos com demanda por kits didáticos integrando hardware e software em tempo real.
4. **Eventos e Feiras de Tecnologia Acadêmica:** Público técnico e visitantes interessados em aplicações de robótica móvel.

### Membros da Equipe

| **Nome** | **Matrícula** | **Curso** | **E-mail** | **Funções** |
|----------|---------------|-----------|------------|-------------|
| [⁠Angel Daniel Grau Barreto](https://github.com/AngelDanielGrau) | 241025158 | Aeroespacial | angeldanielgrau@gmail.com | Estrutura |
| [⁠Angélica]( https://github.com/angelicaccampos ) | 221031256 | Software | angelicadacostacampos@gmail.com | Software |
| [⁠Ana Beatriz Souza Araújo](https://github.com/AnnaBeatrizAraujo) | 241025891 | Software | ana.araujo4519@gmail.com | Software |
| [⁠Ana Caroline](https://github.com/iicaroll) | 241025908 | Software | caroldantas211105@gmail.com | Energia |
| [⁠Carolina](https://github.com/carolinabecker ) | 241038450 | Software | carolsmbecker@gmail.com | Energia |
| [⁠Daniel  Henrique da Silva Rodrigues]( https://github.com/danielhdsrodrigues-pixel ) | 241038601 | Eletrônica | danielhdsrodrigues@gmail.com | Eletrônica |
| [⁠Edson](https://github.com/EdsonToppzera) | 242015826 | Software | edsongabrielbb@gmail.com | Software |
| [⁠Gabriel Mota Oliveira](https://github.com/Gabro-MO) | 241011081 | Software | gabriel1990mota@gmail.com | Software | 
| [⁠Giovana Rocha e Silva](https://github.com/GiovanaRocha16) | 242004715 | Software | giovanarocha160@gmail.com | Estrutura |
| [Gustavo Gomes Fornaciari]( https://github.com/GUGOFO ) | 241032519 | Software | gugofogomes@gmail.com | Eletrônica |
| [⁠Julia Japson Campos da Silveira]( https://github.com/juliajapson ) | 241011368 | Energia | juliajapson@gmail.com | Energia |
| [⁠Julia Patricio]( https://github.com/juliapat18 ) | 231027140 | Software | julia.patricio18@gmail.com | Estrutura |
| [⁠Laura Pinheiro Sumienski]( https://github.com/LauraSumi ) | 241025962 | Energia | Laurasumienski1@gmail.com | Energia |
| [⁠Lucas Fujimoto Tokunaga](https://github.com/Lucasft16) | 241025283 | Software | lucaasft16@gmail.com | Eletrônica |
| [⁠Nicolai Bukvar Miketen](https://github.com/NBukvar) |241025345|Eletrônica|nbmiketen@hotmail.com|Estrutura|
| [⁠Rafael Gomes Pereira](https://github.com/rafgpereira) | 222015248 | Software | rafaelgomespereira123@gmail.com | Software |
| [⁠Thomas Augusto Amorim de Araujo](https://github.com/Thomas4ugust0) | 251016027  | Software  | thomasgusto12@gmail.com | Eletrônica |

**Orientador:**

| **Nome** | **Turma** |
|----------|---------------|
| Prof. Lui T. C. Habl | 03 |

## Orçamento estimado (R$)

### 1.1 Tabela de Estimativa por Componente

As estimativas apresentadas foram elaboradas com base no levantamento histórico de custos de projetos de semestres anteriores, aliado a uma pesquisa de preços médios praticados no mercado atual. Cabe ressaltar que esta análise é preliminar e serve como uma diretriz inicial, uma estimativa orçamentária detalhada e específica para os requisitos do nosso projeto será desenvolvida em uma etapa posterior.

| Item / Serviço | Categoria | Preço Base (R$) | Intervalo Estimado (R$) |
| :--- | :--- | :---: | :---: |
| **ESP-32 WROOM-32** | Eletrônico | R$ 50,00 | R$ 45,00 – R$ 57,50 |
| **Ponte H DRV8833** | Eletrônico | R$ 8,90 | R$ 8,00 – R$ 10,50 |
| **Sensores Infravermelho Sharp20** | Eletrônico | R$ 283,00 | R$ 255,00 – R$ 310,00 |
| **Giroscópio MPU6500** | Eletrônico | R$ 25,99 | R$ 23,00 – R$ 30,00 |
| **Sensor VL53L0X** | Eletrônico | R$ 85,00 | R$ 75,00 – R$ 98,00 |
| **Regulador de Tensão MP1584** | Elétrico | R$ 8,90 | R$ 8,00 – R$ 10,50 |
| **Motores N20-6V (Par)** | Elétrico | R$ 151,80 | R$ 135,00 – R$ 170,00 |
| **Bateria Li-ion 7.4V 2000mAh** | Elétrico | R$ 33,26 | R$ 30,00 – R$ 40,00 |
| **Rodas S20 32mm** | Montagem | R$ 94,80 | R$ 85,00 – R$ 105,00 |
| **Roda Boba Esferográfica** | Montagem | R$ 5,90 | R$ 5,00 – R$ 8,00 |
| **Protoboard** | Montagem | R$ 8,50 | R$ 7,50 – R$ 12,00 |
| **Filamento 3D** | Matéria-prima | R$ 90,00 | R$ 60,00 – R$ 100,00 |

### 1.2 Resumo Financeiro
* **Compra Integral (tudo novo):** R$ 844,21 *(Faixa: R$ 810,00 – R$ 920,00)*
* **Custo Reduzido (reaproveitando Motores, ESP32 e Protoboard):** R$ 407,55
* **Estimativa de Frete:** R$ 50,90
* **Total Estimado com Frete (Cenário Reduzido):** R$ 458,45
* **Valor por Integrante (dividido por 18):** **~R$ 25,47 por pessoa**

---

## 2. Duração Estimada (Horas)

### 2.1 Carga Horária Semanal (Por Integrante)
* **Aulas / Estudos:** 4h/semana (2 aulas de 2h)
* **Trabalho Extraclasse:** 2h/semana
* **Total Semanal por Pessoa:** 6 horas/semana

### 2.2 Capacidade Total do Projeto (18 Integrantes x 14 Semanas)
* **Por Integrante:** 6h/semana x 14 semanas = 84 horas
* **Total do Grupo:** 84h x 18 integrantes = 1.512 x horas

### 2.3 Divisão de Horas por Atividade
1. **Estudos e Aulas (40%):** 604,8 horas
2. **Programação / Firmware (25%):** 378,0 horas
3. **Hardware e Modelagem 3D (20%):** 302,4 horas
4. **Testes e Documentação (15%):** 226,8 horas