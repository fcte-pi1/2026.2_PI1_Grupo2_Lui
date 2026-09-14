# Declaração de Requisitos

A declaração de requisitos compreende o processo formal de identificação, elicitação, análise e documentação das necessidades, restrições e expectativas dos *stakeholders* para a concepção da solução robótica **MicroMouse**.

> **Origem dos Requisitos:**  
> Os requisitos mapeados nesta seção foram formulados a partir de reuniões periódicas e alinhamentos com o orientador do projeto, **Prof. Lui**, aliados à análise técnica do domínio do ecossistema MicroMouse e discussões internas conduzidas pela equipe de desenvolvimento.

---

## Categorização dos Requisitos

Para garantir o rastreamento adequado e a priorização contínua durante o ciclo de desenvolvimento, os requisitos foram categorizados em duas frentes fundamentais:

* **Requisitos Funcionais (RF):** Definem os comportamentos observáveis, operações e serviços diretos que o robô e suas interfaces de suporte devem executar (o *que* o sistema faz).
* **Requisitos Não Funcionais (RNF):** Estabelecem os critérios de qualidade, restrições de *hardware*, parâmetros operacionais, métricas de desempenho e conformidades técnicas exigidas (como o sistema *deve operar*).

---

## Metodologia de Priorização: MoSCoW

A priorização dos requisitos mapeados adota a técnica **MoSCoW**, uma estrutura amplamente empregada no gerenciamento ágil de projetos para alinhar expectativas, otimizar a alocação de recursos e gerenciar o escopo de entrega da solução robótica. 

A sigla **MoSCoW** representa quatro categorias distintas de prioridade:

### 1. **Must Have** 
São os requisitos **críticos e vitais** para a operação mínima viável do robô MicroMouse. Sem a implementação de um requisito *Must Have*, o projeto é considerado inviável ou incapaz de cumprir seu propósito básico.
* **Impacto da ausência:** O robô não funciona ou a entrega falha completamente.

### 2. **Should Have** 
Representam requisitos de **alta relevância** que agregam grande valor à solução, mas cuja ausência na primeira versão não impede completamente o funcionamento básico do robô. Podem existir soluções de contorno temporárias caso sua implementação precise ser postergada.
* **Impacto da ausência:** A eficiência, precisão ou usabilidade do sistema é reduzida, mas a solução continua operacional.

### 3. **Could Have** 
São requisitos **complementares ou diferenciais**, frequentemente categorizados como melhorias técnicas ou funcionalidades secundárias. Eles só serão desenvolvidos caso os requisitos *Must Have* e *Should Have* já estejam estabilizados e haja disponibilidade de tempo, orçamento e recursos.
* **Impacto da ausência:** Nenhum impacto crítico no desempenho principal do robô; afeta apenas funcionalidades extras ou estéticas.

### 4. **Won't Have (this time)** 
Trata-se de requisitos identificados e analisados, mas formalmente acordados como **fora do escopo** para o ciclo atual de desenvolvimento ou competição. Eles permanecem documentados no *backlog* para reavaliação em iterações ou gerações futuras do MicroMouse.
* **Impacto da ausência:** Permite que a equipe mantenha o foco no escopo viável e respeite os prazos do projeto.