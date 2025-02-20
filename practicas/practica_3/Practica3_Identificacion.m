%Practica Identificacio
close all; clc; clear;
load('practica3_ident.mat');
Ts = 1;
y = h(3:end);
x = [(h(2:end-1)) (u(2:end-1))];

%x = [(h(1:end-1)) (u(1:end-1))];
K = h(1)-h(end);
p = 1/352;
e_ojo = K*exp(-p.*t)+ h(end);
sys_ojo = zpk([], [-p], -K)
sys_ojoZOH = c2d(sys_ojo, Ts,'zoh')
%p_ojo = 0.0028
%k = 0.0839

figure()
plot(t,e_ojo,'LineWidth',3);
hold on
plot(t,h,'LineWidth',2);
legend('h estimada','h medida');
xlabel('t [seg]')
hold off
grid

%ZOH
alpha = pinv(x)*y;
%Ad = e^-p
%Bd = k/p * (e^-p -1)
p_disc = -log(alpha(1));
K_disc = p_disc * alpha(2) / (alpha(1)-1);
%p_ojo = 0.0028
%k = 0.0839

sysdisc = ss(alpha(1),alpha(2),1,0,Ts);
sysdisc_ = zpk(sysdisc)
[y_est,t,x] = lsim(sysdisc,(u-u(1)),t,h(1));

figure()
plot(t,y_est,'LineWidth',3);
hold on
plot(t,h,'LineWidth',2);
legend('h estimada','h medida');
xlabel('t [seg]')
hold off
grid

%%
S = 
