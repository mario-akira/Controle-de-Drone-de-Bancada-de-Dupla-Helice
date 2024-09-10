# Controle-de-Drone-de-Bancada-de-Dupla-Hélice

Durante um projeto de graduação foi feito o projeto para o controle de um drone de dupla hélice de bancada didático.

Link para visualização do sistema em funcionamento:https://youtu.be/tLVo63yABtQ

![image](https://github.com/user-attachments/assets/0d3cc475-eb29-4889-ab2c-14c0d187b2a9)

Través de um Stm32f446re e de um potênciometro posicionado no eixo da haste para a determinação do ângulo da haste e então a atuação de controle do sistema.

Para modelagem, foram utilizados dados de um ensaio realizado pelo professor ministrante da disciplina, para um fitting pelo Matlab. A partir da função transferencia obtida, o sistema no domínio da frequência foi passado para o domínio discreto por um segurador de ordem zero, para então permitir o desenvolvimento de um controlador para o ARM.

![image](https://github.com/user-attachments/assets/a4da5af4-0630-4b0f-aa12-8471ea1101e4)

A partir do controlador projetado foi possível aplica-lo no ARM e observar o seu funcionamento analisando o sinal do potenciometro(ângulo do eixo da haste)
