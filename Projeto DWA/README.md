# Navegação Autônoma de Robô Aspirador - Método DWA

Este repositório contém a implementação de um algoritmo de navegação autônoma e desvio de obstáculos para um robô móvel com tração diferencial, utilizando o software de simulação **CoppeliaSim**. O projeto foca na aplicação simplificada do **Método da Janela Dinâmica (Dynamic Window Approach - DWA)**.

## 🚀 Sobre o Projeto

O objetivo deste trabalho é permitir que um robô aspirador explore um ambiente fechado de forma autônoma, evitando colisões com paredes e objetos pequenos através da leitura de sensores de proximidade.

### Especificações Técnicas
* **Modelo do Robô:** Base de tração diferencial integrada no arquivo `BASE_FUNCIONAL_INVISIVEL.ttt`.
* **Atuadores:** 2 Motores de revolução (`MOTOR_ESQUERDO` e `MOTOR_DIREITO`).
* **Sensores:** Conjunto de 5 sensores de proximidade ultrassônicos (`SENSOR_MEIO`, `SENSOR_DIREITO`, `SENSOR_ESQUERDO`, `SENSOR_DIAG_ESQUERDO`, `SENSOR_DIAG_DIREITO`) posicionados para cobrir o campo de visão frontal e lateral.
* **Linguagem:** Python 3.x.
* **API de Comunicação:** Legacy Remote API.

## 🛠️ Requisitos

Para rodar este projeto, você precisará do **CoppeliaSim** instalado e dos seguintes arquivos da API na mesma pasta do script Python:
* `sim.py`
* `simConst.py`

## 💻 Como Rodar

1. Abra o arquivo de cena `BASE_FUNCIONAL_INVISIVEL.ttt` no CoppeliaSim.
2. Certifique-se de que o **Child Script** do robô contenha o comando para iniciar o servidor:
   ```lua
   simRemoteApi.start(19999)
