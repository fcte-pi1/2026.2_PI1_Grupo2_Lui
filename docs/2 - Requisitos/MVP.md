# Definição e Escopo do MVP

Este documento descreve a conceituação, os critérios de inclusão e o conjunto delimitado de requisitos que compõem o **Produto Mínimo Viável (MVP)** do projeto **MicroMouse**.

---

## O que é o MVP?

O **MVP** é a versão mais enxuta de um produto que reúne o conjunto fundamental de funcionalidades necessárias para validar sua proposta de valor principal. No contexto da engenharia e da robótica do **MicroMouse**, o MVP representa o protótipo funcional capaz de percorrer e resolver o labirinto de forma autônoma, garantindo a coleta de dados e operando sob as restrições essenciais de *hardware* e segurança.

O objetivo do MVP não é entregar a solução final com todas as otimizações possíveis, mas sim garantir uma base estável, testável e operacional para validações em cenários reais.

---

## Critérios de Elegibilidade do MVP

A seleção dos requisitos integrantes do MVP baseou-se rigorosamente na metodologia de priorização **MoSCoW**:

* **Critério de Entrada:** Apenas os requisitos categorizados como **Must Have** integram o escopo do MVP.
* **Justificativa:** Os itens *Must Have* representam capacidades indispensáveis sem as quais a missão do robô torna-se inviável. Requisitos classificados como *Should Have*, *Could Have* ou *Won't Have* foram postergados para iterações posteriores.

---

## Requisitos Funcionais do MVP

A tabela a seguir apresenta os Requisitos Funcionais essenciais selecionados para o MVP. O ID de cada requisito fornece um link direto para o arquivo de origem e a linha exata de sua declaração.

| **ID** | **Nome do Requisito** |
|:------:|------------------------|
| [RF01](RF.md#L14) | Mapear Labirinto |
| [RF02](RF.md#L15) | Exibir dados do Labirinto |
| [RF03](RF.md#L16) | Exibir dados do MicroMouse |
| [RF04](RF.md#L17) | Identificar Paredes |
| [RF05](RF.md#L18) | Monitorar Localização |
| [RF06](RF.md#L19) | Transmitir dados |
| [RF07](RF.md#L20) | Fazer giro completo |
| [RF08](RF.md#L21) | Andar para frente |
| [RF09](RF.md#L22) | Armazenar dados |
| [RF10](RF.md#L23) | Garantir integridade |
| [RF11](RF.md#L24) | Navegar labirinto |
| [RF12](RF.md#L25) | Realizar telemetria |
| [RF13](RF.md#L26) | Garantir autonomia |

---

## Requisitos Não Funcionais do MVP

A tabela abaixo compõe as restrições técnicas, parâmetros elétricos e de desempenho que o MVP deve atender obrigatoriamente.

| **ID** | **Requisito Não Funcionak** |
|:------:|---------------|
| [RNF01](RNF.md#L13) | Os dados da execução devem ser armazenados de forma persistente para consultas posteriores. | 
| [RNF02](RNF.md#L14) | A bateria deve ter uma capacidade suficiente para realizar $3$ trajetos ou $30\text{ minutos}$ de operação contínua. |
| [RNF03](RNF.md#L15) | Deve ter uma bateria recarregável. |
| [RNF04](RNF.md#L16) | O robô deve operar em corredores formados por células de $18\text{ cm}^2$. |
| [RNF05](RNF.md#L17) | O sistema de controle de motores deve garantir que a velocidade e a frenagem impeçam o robô de colidir com as paredes com força suficiente para causar danos físicos à estrutura do labirinto. |