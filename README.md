# Navegação Híbrida no CoppeliaSim (A* + DWA)

Este projeto desenvolvido para a disciplina de Robótica implementa um sistema de navegação autônoma para um robô móvel diferencial simulado no CoppeliaSim. O sistema integra mapeamento do ambiente por visão computacional, planejamento global de rota utilizando o algoritmo A* e controle reativo local via Dynamic Window Approach (DWA). O objetivo é permitir que o robô navegue de forma autônoma até um alvo, desviando de obstáculos de forma segura.

## 🚀 Como Executar o Projeto

### Pré-requisitos
Certifique-se de ter os seguintes softwares e dependências instalados:
* **CoppeliaSim** (com suporte configurado para a ZeroMQ Remote API)
* **Python 3.x**
* Bibliotecas Python necessárias:
  ```bash
  pip install numpy matplotlib coppeliasim-zmqremoteapi-client
