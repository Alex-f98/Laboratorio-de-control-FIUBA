clear; clc; close all;
data = load('practica3_ident.mat');
Ts = 1;

%% 1)

p = 1/352;
h_end = data.h(end);
k = -(data.h(1) - h_end);

data_stim = -k*exp( -(p*data.t) ) + h_end;
figure(1);
hold on;
plot(data.t, data_stim, 'LineWidth', 3)
plot(data.t, data.h,  'LineWidth', 2)


%% 2)
syms k p s real
%s = tf('s')
%P = -k/(s+p)
%
%Ad = e^-p
%Bd = k/p * (e^-p -1)

h = data.h(1:end) - data.h(1);
u = data.u(1:end) - data.u(1);
t = data.t(1:end);

X = [h(2:end-1), u(2:end-1)];
Y = h(3:end);

param = pinv(X)*Y;
%param = inv(X'*X)*X'*Y;

%p_ojo = 0.0028
%k = 0.0839

p_lq = -log(param(1));
k_lq = (param(2)*p_lq)/(exp(-p_lq)-1);

%%
sysdisc = ss(param(1),param(2), 1, 0, Ts);
sysdisc_ = zpk(sysdisc)
[y_est, t, x] = lsim(sysdisc,(u-u(1)),t,h(1));

figure(2)
plot(t, data_stim,'LineWidth',3)
hold on
plot(t, data.h(1) + y_est,'LineWidth',3);
plot(t, data.h(1) + h,'LineWidth',2);
legend('h estimada visualmente','h estimada por LQ','h medida');
xlabel('t [seg]')
hold off
grid

%%
% data_stim_lq = -k_lq*exp( -(p_lq*data.t(3:end)) ) + h_end;
% %data_stim_lq = X*param
% figure(2);
% hold on;
% %plot(data.t(3:end), data_stim(3:end))
% plot(data.t(3:end), data.h(3:end))
% plot(data.t(3:end), data_stim_lq)
% legend('h medida', 'h estimada LQ');

%%
h_0 = 0.45;
g = 9.8; L = 0.9; l_2 = 0.4; l_1 = 0.1;
K_ojo = 0.0839;


%-K_ojo/(s+p)
%K = (S*sqrt(2*g*h0))/(l_1 + h0*(l_2-l_1)/(L))^2
S_ojo = k_lq*( l_1 + h_0*(l_2-l_1)/L )^2 /sqrt(2*g*h_0);

%mis datos.
Diametro = 10.65/1000;      %[m]     Diametro de la cañeria de salida
S = (pi*Diametro^2)/4;
disp("S visual"); disp(S_ojo)
disp("S"); disp(S)
%%
p_ = 0.00237;
Ad = exp(-p_);
X_ = u(2:end-1);
Y_ = h(3:end) - Ad*h(2:end-1);
Bd = pinv(X_)*Y_;
%param = inv(X'*X)*X'*Y;

%p_ojo = 0.0028
%k = 0.0839
p_lq = -log(Bd);

sysdisc = ss(Ad,Bd, 1, 0, Ts);
sysdisc_ = zpk(sysdisc)
[y_est3, t, x3] = lsim(sysdisc,(u-u(1)),t,h(1));

figure(3)
plot(t, data_stim,'LineWidth',3)
hold on
plot(t, data.h(1) + y_est,'LineWidth',3);
plot(t, data.h(1) + y_est3,'LineWidth',3);
plot(t, data.h(1) + h,'LineWidth',2);
legend('h estimada visualmente','h estimada por LQ', 'h, estimando K por LQ','h medida');
xlabel('t [seg]')
hold off
grid

%K_lq3 = -Bd;
K_lq3 = (Bd*p_)/(exp(-p_)-1);
%-K_ojo/(s+p)
%K = (S*sqrt(2*g*h0))/(l_1 + h0*(l_2-l_1)/(L))^2
S_lq3 = K_lq3*( l_1 + h_0*(l_2-l_1)/L )^2 /sqrt(2*g*h_0);

%mis datos.
Diametro = 10.65/1000;      %[m]     Diametro de la cañeria de salida
S = (pi*Diametro^2)/4;
disp("S lq3"); disp(S_lq3)
disp("S"); disp(S)


