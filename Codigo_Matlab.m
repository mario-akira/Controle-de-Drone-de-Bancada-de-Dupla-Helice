%%
%Planta sem dividir por 15
s = tf('s')
Planta = (125.2/(s-0.1295))-(15.36/s^2)-124.8/s
%step(Planta)
%%
%Planta dividindo por 15
s = tf('s')
Planta_Oficial = (0.4044/(s+0.059))+(0.3604/s^2)-0.3689/s
%%
%Planta Discretizada Zoh
Planta_Digital = c2d(Planta_Oficial,20e-3,'zoh')
%%
%Sistema_Discretizado_com_Controlador
%Tem q extrair do sisotool
figure
step(feedback(series(Planta_Digital,C),1))
%%
%Sinal de controle
U = C/(1+Planta_Digital*C);
figure
step(U)
%%