%Practica 6 - Balancio 
s = tf('s');

P = -0.099863*s /((s + 10.14)*(s - 10.14));

P2 = tf([-0.09986 0],[1 0 -102.8]);
N = 1000
kp = -240;
ki = -2.28e3;
Td = -1;



%%
kp = -240;
ki = -2.28e3;
Td = -1;


Ti = 10.14
C = kp*(1 + 1/(Ti*s))
%C = kp*(1 + 1/(Ti*s) + Td*s)
L = C*P

figure(1); bode(P, L, 1/s)
%sys = feedback(C*P, 1);
%Ts = 0.01;
%C_disc = c2d(C,Ts,'tustin');