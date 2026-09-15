# Definição e Escopo do MVP

Este documento descreve a conceituação, os critérios de inclusão e o conjunto delimitado de requisitos que compõem o **Produto Mínimo Viável (MVP)** do projeto **MicroMouse**.

---

## O que é o MVP?

O **MVP** é a versão básica de um produto que reúne o conjunto fundamental de funcionalidades necessárias para validar sua proposta de valor principal. No contexto da engenharia e da robótica do **MicroMouse**, o MVP representa o protótipo funcional capaz de percorrer e resolver o labirinto de forma autônoma, garantindo a coleta de dados e operando sob as restrições essenciais de *hardware* e segurança.

O objetivo do MVP não é entregar a solução final com todas as otimizações possíveis, mas sim garantir uma base estável, testável e operacional para validações em cenários reais.

---

## Critérios de Elegibilidade do MVP

A seleção dos requisitos integrantes do MVP baseou-se rigorosamente na metodologia de priorização **MoSCoW**:

* **Critério de Entrada:** Apenas os requisitos categorizados como **Must Have** integram o escopo do MVP.
* **Justificativa:** Os itens *Must Have* representam capacidades indispensáveis sem as quais a missão do robô torna-se inviável. Requisitos classificados como *Should Have*, *Could Have* ou *Won't Have* foram postergados para iterações posteriores.

---

## Requisitos Funcionais do MVP

A tabela a seguir apresenta os Requisitos Funcionais essenciais selecionados para o MVP. O ID de cada requisito fornece um link direto para o arquivo de origem.

| **ID** | **Nome do Requisito** | **Descrição** |
|:------:|------------------------|---------------|
| [RF01](RF.md#rf-declarados) | Mapear Labirinto | Realizar o mapeamento contínuo da estrutura do labirinto à medida que o robô se desloca. |
| [RF02](RF.md#rf-declarados) | Exibir dados do Labirinto | Apresentar em interface os dados estruturais e o mapa gerado do labirinto. |
| [RF03](RF.md#rf-declarados) | Exibir dados do MicroMouse | Exibir em tempo real as informações de status, parâmetros e estado do robô MicroMouse. |
| [RF04](RF.md#rf-declarados) | Identificar Paredes | Detectar a presença e a proximidade de paredes adjacentes ao robô. |
| [RF05](RF.md#rf-declarados) | Monitorar Localização | Acompanhar e atualizar a posição exata do robô dentro da grade do labirinto. |
| [RF06](RF.md#rf-declarados) | Transmitir dados | Enviar os dados coletados pelos sensores e sistema de bordo para a central de controle. |
| [RF07](RF.md#rf-declarados) | Fazer giro completo | Executar a rotação controlada de retorno (giro completo/180°) no próprio eixo. |
| [RF08](RF.md#rf-declarados) | Andar para frente | Executar o deslocamento linear para a frente ao longo das células do labirinto. |
| [RF09](RF.md#rf-declarados) | Armazenar dados | Registrar e salvar localmente as informações operacionais coletadas durante a navegação. |
| [RF10](RF.md#rf-declarados) | Navegar labirinto | Executar os algoritmos de busca e tomada de decisão para percorrer autonomamente o labirinto. |
| [RF11](RF.md#rf-declarados) | Realizar telemetria | Coletar e transmitir métricas de operação do sistema para acompanhamento contínuo. |

---

## Requisitos Não Funcionais do MVP

A tabela abaixo compõe as restrições técnicas, parâmetros elétricos e de desempenho que o MVP deve atender obrigatoriamente.

| **ID** | **Requisito Não Funcional** |
|:------:|---------------|
| [RNF01](RNF.md#rnf-declarados) | Armazenar os dados da execução de forma persistente para consultas posteriores. |
| [RNF02](RNF.md#rnf-declarados) | Garantir que a bateria tenha capacidade suficiente para realizar 3 trajetos ou 30 minutos de operação contínua. |
| [RNF03](RNF.md#rnf-declarados) | Possuir bateria recarregável. |
| [RNF04](RNF.md#rnf-declarados) | Operar o robô em corredores formados por células de 18 cm². |
| [RNF05](RNF.md#rnf-declarados) | Garantir por meio do sistema de controle de motores que a velocidade e a frenagem impeçam o robô de colidir com as paredes com força suficiente para causar danos físicos à estrutura do labirinto. |
| [RNF06](RNF.md#rnf-declarados) | Assegurar que o robô se mantenha intacto no final do trajeto. |