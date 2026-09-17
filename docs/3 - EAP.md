# Estrutura Analítica do Projeto (EAP)

Este documento apresenta a especificação técnica e a fundamentação metodológica da **Estrutura Analítica do Projeto (EAP)** desenvolvida para o projeto **MicroMouse**, alinhada rigorosamente às diretrizes acadêmicas da disciplina e às boas práticas de Gerenciamento de Projetos.

---

## 1. Fundamentos

Conforme a definição estabelecida para a disciplina:

> *A EAP representa a decomposição hierárquica do escopo total do trabalho a ser executado pela equipe do projeto a fim de alcançar os objetivos e criar entregas exigidas.* 

### Diretrizes e Regras de Elaboração
A EAP desenvolvida adota a abordagem por **Estrutura Analítica de Produto (EAP por Entregas)**. Para garantir a aderência às especificações do projeto e aos padrões formais, a estrutura atende rigorosamente às seguintes diretrizes:

1. **Exclusividade de Entregas e Pacotes de Trabalho:** A EAP contempla **somente entregas e pacotes de trabalho**. Todos os elementos são representados por substantivos ou produtos concluídos.
2. **Sem Associação às Fases do Projeto:** A EAP não é dividida pelas fases do ciclo de vida. A decomposição é 100% orientada aos componentes e sub-entregas do produto .
3. **Sem Atividades:** Atividades, tarefas e verbos de ação pertencem ao cronograma do projeto e foram totalmente excluídos da EAP .
4. **Estrutura Hierárquica Pai-Filho:** Relação estritamente vertical e paralela do todo para as partes, sem ligações em série que representem fluxogramas ou sequenciamento cronológico .
5. **Regra dos 100%:** A soma de todos os pacotes de trabalho do nível inferior cobre 100% do escopo do nó pai, sem omissões e sem contabilizar nenhum trabalho duas vezes.
6. **Dimensionamento dos Pacotes:** Cada pacote de trabalho no nível mais baixo representa um esforço estimado entre 8 e 80 horas de trabalho.

---

## 2. EAP Completo

A EAP geral do projeto **MicroMouse** organiza todo o escopo do sistema no Nível 0 ("MicroMouse"), decompondo-o hierarquicamente em 5 módulos de entrega principais no Nível 1: Documentação, Sistema de Software Embarcado, Sistema de Energia Elétrica, Conjunto Mecânico e Estrutural, e Sistema Eletrônico Embarcado.

<iframe style="border: 1px solid rgba(0, 0, 0, 0.1);" width="800" height="450" src="https://embed.figma.com/board/7VrB5aneHO1BiXhVx7DALM/EAP?node-id=0-1&embed-host=share" allowfullscreen></iframe>

![EAP Completo](./figs/EAP/EAP.png)

### Visão Geral da Decomposição por Entregas
- **Nível 0:** MicroMouse
- **Nível 1 (Módulos de Entrega do Produto / Subsistemas):**
  - **Documentação:** Artefatos formais de engenharia, gestão e relatórios de validação.
  - **Sistema de Software Embarcado:** Inteligência lógica, algoritmos de navegação, comunicação e interface do robô.
  - **Sistema de Energia Elétrica:** Subsistemas de alimentação, regulação, proteção e autonomia energética.
  - **Conjunto Mecânico e Estrutural:** Modelagem CAD, chassi físico, elementos de locomoção e pista de testes.
  - **Sistema Eletrônico Embarcado:** Placas de circuito impresso (PCB/protoboard), microcontrolador e sensoriamento.

---

## 3. Documentação

O módulo de **Documentação** consolida todos os artefatos formais de engenharia, planos de gestão e relatórios consolidados de testes do projeto.

![EAP Documentação](./figs/EAP/Documentacao.png)

### Estrutura de Entregas e Pacotes de Trabalho
- **Documentação**
  - **Especificação Técnica**
    - Especificação de Requisitos
    - Projeto Conceitual ( Estruturas )
    - Projeto Conceitual ( Energia )
    - Projeto Conceitual ( Hardware )
    - Projeto Conceitual ( Software )
  - **Gestão e Planejamento**
    - Termo de Abertura do Projeto
    - Relatorio de Cronograma
    - Relatorio de Cronograma
    - Avaliação de Desempenho
    - Relatorio Unificado de Testes
    - Relatório de encerramento do Projeto

---

## 4. Sistema de Software Embarcado

O módulo **Sistema de Software Embarcado** especifica os pacotes de trabalho referentes à lógica de controle, comunicação, armazenamento e visualização em tempo real.

![EAP Software](./figs/EAP/Software.png)

### Estrutura de Entregas e Pacotes de Trabalho
- **Sistema de Software Embarcado**
  - **Firmware Embarcado**
    - Interface de sensores e motores
    - Módulo de Mapeamento e Navegação
    - Módulo de Telemetria Embarcada
  - **Comunicação e Processamento**
    - Receptor de Telemetria
    - Módulo de Integração
  - **Armazenamento de Dados**
    - Histórico de Corridas e Eventos
    - Banco de Dados Operacional
  - **Interface de Usuário**
    - Painel do Mapa em Tempo Real
    - Painel de Status do Robô
  - **Teste e Avaliação**
    - Relatório de Testes Funcionais

---

## 5. Sistema de Energia Elétrica

O módulo **Sistema de Energia Elétrica** organiza as entregas e pacotes de trabalho referentes ao fornecimento, distribuição, segurança e testes de eficiência energética.

![EAP Energia](./figs/EAP/Energia.png)

### Estrutura de Entregas e Pacotes de Trabalho
- **Sistema de Energia Elétrica**
  - **Planejamento Energético**
    - Relatório de Consumo dos Componentes
    - Memorial de Cálculo de Autonomia e Tensão
  - **Subsistema de Alimentação**
    - Documento de Especificação da Bateria
    - Banco de Baterias
    - Módulo de Carregamento
  - **Subsistema de Distribuição de Energia**
    - Circuito de distribuição
    - Reguladores de tensão
    - Conectores e interligações
  - **Subsistema de Proteção e Controle**
    - Módulo de Proteção contra Sobrecorrente
    - Módulo de Proteção contra Curto-Circuito
    - Módulo de Proteção contra Descarga Excessiva
    - Circuito de Interrupção dos Motores
    - Interface de Controle com a Eletrônica
  - **Validação dos Requisitos**
    - Relatório de Teste de Autonomia
    - Relatório de Teste de Recarga
    - Relatório de Teste de Interrupção dos Motores
    - Documentação Técnica Consolidada

---

## 6. Sistema Eletrônico Embarcado

O módulo **Sistema Eletrônico Embarcado** compreende a infraestrutura física de hardware, sensores, atuadores e montagem dos circuitos elétricos.

![EAP Eletrônica](./figs/EAP/Eletronica.png)

### Estrutura de Entregas e Pacotes de Trabalho
- **Sistema Eletrônico Embarcado**
  - **Hardware**
    - Telemetria
    - Microcontrolador
    - Sistema Locomotor
    - Sensoriamento
  - **Circuitos da Placa (Protoboard)**
    - Esquemático
    - Lista de Componentes
    - Circuito Montado

---

## 7. Conjunto Mecânico e Estrutural

O módulo **Conjunto Mecânico e Estrutural** reúne as entregas de engenharia mecânica, chassi físico, elementos de locomoção, testes e a pista de validação do robô.

![EAP Estrutura](./figs/EAP/Estrutura.png)

### Estrutura de Entregas e Pacotes de Trabalho
- **Conjunto Mecânico e Estrutural**
  - **CAD**
    - Desenho Técnico
    - Relatório do Teste de Arranjo
    - Relatório de Análise de Massa e CG
    - Modelo 3D da Estrutura
  - **Chassi**
    - Estrutura Principal
    - Lista de Materiais Especificados
    - Encaixes para dispositivos
    - Suporte para atuadores
  - **Elementos de Locomoção**
    - Rodas e pneus
    - Eixos
    - Sistema de Estabilidade
  - **Pista de Testes**
    - Base da pista
    - Paredes
    - Acabamento visual e funcional
    - Pista de Testes Validada
  - **Testes Estruturais**
    - Relatório de Verificação Dimensional
    - Relatório de Teste de Resistência
    - Relatório de Teste de Estabilidade
    - Documentação da Validação Estrutural