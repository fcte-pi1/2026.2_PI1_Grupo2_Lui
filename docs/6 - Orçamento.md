# Orçamento

- **Última atualização:** 30/09/2026.
- **Coluna "Realizado":** deve ser preenchida com o valor efetivamente pago, conforme as compras forem feitas.

## 1. Resumo

| Indicador | Valor (R$) |
|-----------|-----------:|
| Mão de obra (estudantes, valor de referência) | 104.192,20 |
| Serviços | 50,90 |
| Equipamentos e materiais | 876,04 |
| **Custo total do projeto** | **105.119,14** |
| Desembolso em dinheiro (serviços + materiais) | 926,94 |
| Desembolso por integrante (19 integrantes) | 48,79 |

A mão de obra não é gasto em dinheiro: entra no orçamento como valor de referência do tempo dos estudantes. O que a equipe paga de fato é o desembolso de R$ 926,94.

## 2. Orçamento consolidado

| **Item de Custo** | **Qtde.** | **Previsto (R$)** | **Realizado (R$)** |
|:------------------|----------:|------------------:|-------------------:|
| <span style="color: yellow;">**Mão de Obra**</span> | | 104.192,20 | |
| Estudante (custo/hora de um estudante da UnB: R$ 39,17) | 2.660 h | 104.192,20 | |
| <span style="color: yellow;">**Serviços**</span> | | 50,90 | |
| Impressão 3D (Faculdade UnB Gama) | 1 | 0,00 | |
| Marcenaria (corte de madeira, equipamentos próprios) | 1 | 0,00 | |
| Frete das compras online | 1 | 50,90 | |
| <span style="color: yellow;">**Equipamentos e Materiais**</span> | | 876,04 | |
| *Eletrônica* | | 303,22 | |
| ESP32 DevKit 30 pinos | 2 | 79,98 | |
| Ponte H DRV8833 | 1 | 16,00 | |
| Motor N20 6V com encoder | 2 | 57,78 | |
| Sensor ultrassônico HC-SR04 | 4 | 75,00 | |
| Giroscópio IMU MPU6500 | 1 | 11,89 | |
| Tradutor de nível lógico | 1 | 19,00 | |
| Protoboard 400 pontos | 1 | 19,43 | |
| Kit de jumpers (120 un) | 1 | 24,14 | |
| *Energia* | | 131,29 | |
| Bateria recarregável 7,4 V 3300 mAh (18650) | 1 | 33,25 | |
| Cabo conector JST XH 2,54 mm, 2 pinos, 20 cm | 1 | 20,80 | |
| Regulador buck MP1584 | 1 | 41,34 | |
| Medidor de tensão/corrente INA226 | 1 | 35,90 | |
| *Estrutura: carrinho* | | 185,21 | |
| Chassi, plataforma, suportes e cubo eixo-rodas (ABS Premium) | 1 | 124,69 | |
| Rodas motrizes | 2 | 24,65 | |
| Roda boba (ball caster) | 1 | 35,87 | |
| *Estrutura: labirinto* | | 206,32 | |
| MDF para o labirinto | 1 | 186,57 | |
| Cola para madeira | 1 | 8,00 | |
| Lixa para madeira | 1 | 11,75 | |
| Suportes impressos (reaproveitamento do ABS) | 1 | 0,00 | |
| *Estrutura: extras* | | 50,00 | |
| Porcas, parafusos, arruelas e buchas | 1 lote | 50,00 | |
| <span style="color: blue;">**TOTAL**</span> | | **105.119,14** | |

## 3. Mão de obra

### Valor da hora do estudante

O valor da hora do estudante da UnB é de **R$ 39,17**. O levantamento que originou esse valor usa a versão mais atualizada (2024) do Anuário Estatístico da UnB, referente ao ano letivo de 2023, combinada com dados da LOA 2025 (Lei Orçamentária Anual), do MEC (Ministério da Educação) e do DPO (Decanato de Planejamento, Orçamento e Avaliação Institucional).

### Horas do projeto

Cada integrante dedica 10 h/semana (4 h de aula + 6 h extraclasse) durante 14 semanas, o que dá 140 h por integrante. A R$ 39,17/h, isso corresponde a R$ 5.483,80 por integrante.

| Área | Integrantes | Horas | Custo (R$) |
|:-----|------------:|------:|-----------:|
| Estrutura | 4 | 560 | 21.935,20 |
| Software | 6 | 840 | 32.902,80 |
| Eletrônica | 5 | 700 | 27.419,00 |
| Energia | 4 | 560 | 21.935,20 |
| **Total** | **19** | **2.660** | **104.192,20** |

## 4. Detalhamento das compras

### Eletrônica e Energia

| Item | Modelo | Qtde. | Valor unit. (R$) | Previsto (R$) | Onde comprar |
|:-----|:-------|------:|-----------------:|--------------:|:-------------|
| ESP32 | DevKit 30 pinos | 2 | 39,99 | 79,98 | [Shopee](https://shopee.com.br/Placa-Esp32-Wifi-Bluetooth-30-38-Pinos-soldados-i.322895481.23893449076) |
| Ponte H | DRV8833 | 1 | 16,00 | 16,00 | [Mercado Livre](https://www.mercadolivre.com.br/ponte-h-dupla-2-canais-drv8833-2a-placa-preta/p/MLB35859089) |
| Motor | N20 6V com encoder | 2 | 28,89 | 57,78 | [AliExpress](https://pt.aliexpress.com/item/1005010436307480.html) |
| Sensor ultrassônico | HC-SR04 | 4 | 18,75 | 75,00 | [Mercado Livre](https://www.mercadolivre.com.br/kit-com-2-unidades-sensor-ultrassonico-hc-sr04-casa-da-robotica-5v-distancia-arduino/p/MLB33346342) |
| Giroscópio IMU | MPU6500 | 1 | 11,89 | 11,89 | [AliExpress](https://pt.aliexpress.com/item/1005008632918302.html) |
| Tradutor de nível lógico | Conversor bidirecional 5 V / 3,3 V | 1 | 19,00 | 19,00 | [Mercado Livre](https://www.mercadolivre.com.br/conversor-nivel-logico-bidirecional-i2c-5v-p-33v-arduino/up/MLBU1401919651) |
| Protoboard | 400 pontos | 1 | 19,43 | 19,43 | [Mercado Livre](https://www.mercadolivre.com.br/protoboard-breadboard-400-pontos-furos-pinos-unidade/p/MLB2089243318) |
| Jumpers | Kit com 120 un | 1 | 24,14 | 24,14 | [Mercado Livre](https://www.mercadolivre.com.br/kit-cabo-jumper-macho-macho--macho-femea--femea-femea-120x/up/MLBU2299178999) |
| Bateria recarregável | 7,4 V 3300 mAh, modelo 18650 | 1 | 33,25 | 33,25 | [Mercado Livre](https://www.mercadolivre.com.br/p/MLB46386246?pdp_filters=item_id:MLB5024462353) |
| Cabo conector | JST XH 2,54 mm, 2 pinos, 20 cm | 1 | 20,80 | 20,80 | [Mercado Livre](https://www.mercadolivre.com.br/up/MLBU5165009963?pdp_filters=item_id:MLB7640118870) |
| Regulador buck | MP1584 (kit com 5) | 1 | 41,34 | 41,34 | [Mercado Livre](https://www.mercadolivre.com.br/5x-mini-conversor-dc-dc-3a-step-down-mp1584-mp1584en/p/MLB2088650164) |
| Medidor de tensão/corrente | INA226 | 1 | 35,90 | 35,90 | [Mercado Livre](https://www.mercadolivre.com.br/ina226-modulo-sensor-de-corrente-dc-alta-precisao-i2c/p/MLB2046304522) (preço estimado) |
| **Subtotal** | | | | **434,51** | |

### Estrutura

| Grupo | Item | Especificação | Previsto (R$) | Onde comprar |
|:------|:-----|:--------------|--------------:|:-------------|
| Carrinho | Chassi, plataforma, suportes e cubo eixo-rodas | Filamento ABS Premium MG-94, vermelho, 1,75 mm, 1 kg | 124,69 | [Loja National 3D](https://www.lojanational3d.com.br/filamentoabspremiummg-94vermelhoextintor175mm1kg/prod-7395641/) |
| Carrinho | Rodas motrizes | Roda 34 mm para micro motor N20 (kit com 2) | 24,65 | [Mercado Livre](https://www.mercadolivre.com.br/2x-roda-34mm-para-micro-motor-dc-12v-n20-robo/up/MLBU664879255) |
| Carrinho | Roda boba | Esfera transferidora de aço carbono (ball caster) | 35,87 | [Mercado Livre](https://www.mercadolivre.com.br/esfera-transferidora-aco-carbono-roda-boba-para-robotica/up/MLBU2884521198) |
| Labirinto | MDF | MDF cru 12 mm, chapa de 185 x 275 cm, 2 faces (Berneck) | 186,57 | [Madeiranit](https://www.madeiranit.com.br/mdf-cru-12mm-185x275cm-2-faces-berneck) (a definir) |
| Labirinto | Cola para madeira | Tekbond 406729, branca | 8,00 | [Amazon](https://www.amazon.com.br/dp/B078GWC46C) |
| Labirinto | Lixa para madeira | Lixa grão 120 para massa e madeira (Tigre), folha 225 x 275 mm | 11,75 | [Amazon](https://www.amazon.com.br/dp/B0G81SHY4W) |
| Labirinto | Suportes impressos | Reaproveitamento do ABS | 0,00 | |
| Serviços | Impressão 3D | Faculdade UnB Gama | 0,00 | |
| Serviços | Corte de madeira | Equipamentos próprios | 0,00 | |
| Extras | Porcas, parafusos, arruelas e buchas | A definir | 50,00 | |
| **Subtotal** | | | **441,53** | |