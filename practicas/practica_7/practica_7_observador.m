clc;clear;close all;

frec = 100; %hz
L = 0.6;
r2 = 2.625;
g = 9.8;
T = 1/frec;
polos = [0.5 0.6];
%syms L r2 g T real
%syms theta_k w_k b_k  theta_k_obs w_k_obs real
%syms l1 l2 real

%X = [theta_k w_k]'
%LL = [l1 l2]'
Ad = eye(2) + [0 1; -g/L -r2]*T
Cd = [1 0]
Bd = zeros(2,1);

LL = acker(Ad', Cd', polos)

%Xkp1 = Ad*X + LL*(theta_k_obs - Cd*X)

%%
syms l_11 l_12 l_21 l22 l_31 l32 real

%Xk = [X; b_k]

Ad2 = [Ad [0 0]'; 0 0 1]
Cd2 = [Cd 0; 0 1 1]
polos2 = [polos 0.98]

% LL2 = [l_11 l_12;
%        l_21 l22;
%        l_31 l32]
LL2 = place(Ad2', Cd2', polos2)



%Xkp1_ = Ad2*Xk + LL2*([theta_k_obs w_k_obs]' - Cd2*Xk)
