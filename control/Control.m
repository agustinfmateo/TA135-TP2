%% Control clasico
clear all;close all;clc
s = tf('s');
 
%         73.03 s (s-90.01)
%  --------------------------------
%  (s+35.2)^2 (s^2 + 0.97s + 91.63)

% Separo las transferencias
Phi = zpk([], [-35.2, -35.2],  1.2390e+03);

ceros = [0, 90.01];
polos = [-0.4850 + 9.5599i, -0.4850 - 9.5599i];
ganancia = 73.03/1.2390e+03;

Theta = zpk(ceros, polos,  ganancia);

P= Phi*Theta

%% PID
Ts=20e-3;

%Kp = -0.588;
%Ki = -5.6;
%Kd = -0.0028;
%C = pid(Kp, Ki, Kd)
%T = zpk(C)

k=-0.45;

C = k;                                %P
%C = k*(s+50)                          %PD
%C = k*(s+6)/s                         %PI
%C = k*(s+10)*(s+200)/s                %PID
%C = k*(s+35.2)*(s+35.2)/(s*(s+100))   %PID Filtrado

C_pid = pid(C)
[Kp, Ki, Kd, Tf] = piddata(C_pid);

L=minreal(P*C);
S = 1 / (1 + L); % Respuesta del péndulo (qué tan rápido vuelve a 1)
CS = -C / (1 + L); % Respuesta del brazo (esfuerzo del controlador)

C_d = c2d(C, Ts, 'tustin');

%% Root locus de L
figure()
rlocus(L/-k); title("Lugar de las raices de L");

%% Bode
figure;
bode(L);
grid;
title("Bode de L");

%% polos y ceros a lazo cerrado
T = feedback(P*C, 1);
figure;
pzmap(T);
grid on;
title('Polos a lazo cerrado');
grid on;
title('Polos y ceros a lazo cerrado');

%% Gráficos Medición vs Simulación

%% Extraer variables de la medición
vars = out.who;
for i = 1:length(vars)
    assignin('base', vars{i}, out.(vars{i}));
end

%Matlab hizo cosas raras con los tiempos
t_sim_u     = linspace(0, 6, length(u_sim))';
t_sim_phi   = linspace(0, 6, length(phi_sim))';
t_sim_theta = linspace(0, 6, length(theta_sim))';


%% P

%Valores en equilibrio
u=u-90;
phi=phi-90;

figure;
subplot(3, 1, 1);
plot(tout, u, 'b', 'LineWidth', 1.2); 
hold on;
plot(t_sim_u, u_sim, 'r--', 'LineWidth', 1.2);
hold off;
grid on;
xlabel('Tiempo (s)');
ylabel('u');
title('Entrada u');
legend('Real', 'Simulado');
xlim([1 6])

subplot(3, 1, 2);
plot(tout, phi, 'b', 'LineWidth', 1.2); 
hold on;
plot(t_sim_phi, phi_sim, 'r--', 'LineWidth', 1.2);
hold off;
grid on;
xlabel('Tiempo (s)');
ylabel('\phi');
title('Ángulo \phi');
legend('Real', 'Simulado');
xlim([1 6])

subplot(3, 1, 3);
plot(tout, theta, 'b', 'LineWidth', 1.2); 
hold on;
plot(t_sim_theta, theta_sim, 'r--', 'LineWidth', 1.2);
hold off;
grid on;
xlabel('Tiempo (s)');
ylabel('\theta');
title('Ángulo \theta');
legend('Real', 'Simulado');
xlim([1 6])

%% PD
vars = out.who;
for i = 1:length(vars)
    assignin('base', vars{i}, out.(vars{i}));
end

%Matlab hizo cosas raras con los tiempos
t_sim_u     = linspace(0, 6, length(u_sim))';
t_sim_phi   = linspace(0, 6, length(phi_sim))';
t_sim_theta = linspace(0, 6, length(theta_sim))';

phi=phi-90;

figure;
subplot(3, 1, 1);
plot(tout, u, 'b', 'LineWidth', 1.2); 
hold on;
plot(t_sim_u, u_sim, 'r--', 'LineWidth', 1.2);
hold off;
grid on;
xlabel('Tiempo (s)');
ylabel('u');
title('Entrada u');
legend('Real', 'Simulado');
xlim([1 6])

subplot(3, 1, 2);
plot(tout, phi, 'b', 'LineWidth', 1.2); 
hold on;
plot(t_sim_phi, phi_sim, 'r--', 'LineWidth', 1.2);
hold off;
grid on;
xlabel('Tiempo (s)');
ylabel('\phi');
title('Ángulo \phi');
legend('Real', 'Simulado');
xlim([1 6])

subplot(3, 1, 3);
plot(tout, theta, 'b', 'LineWidth', 1.2); 
hold on;
plot(t_sim_theta, theta_sim, 'r--', 'LineWidth', 1.2);
hold off;
grid on;
xlabel('Tiempo (s)');
ylabel('\theta');
title('Ángulo \theta');
legend('Real', 'Simulado');
xlim([1 6])

%% PI
vars = out.who;
for i = 1:length(vars)
    assignin('base', vars{i}, out.(vars{i}));
end

%Matlab hizo cosas raras con los tiempos
t_sim_u     = linspace(0, 6, length(u_sim))';
t_sim_phi   = linspace(0, 6, length(phi_sim))';
t_sim_theta = linspace(0, 6, length(theta_sim))';

phi=phi-90;

figure;
subplot(3, 1, 1);
plot(tout, u, 'b', 'LineWidth', 1.2); 
hold on;
plot(t_sim_u, u_sim, 'r--', 'LineWidth', 1.2);
hold off;
grid on;
xlabel('Tiempo (s)');
ylabel('u');
title('Entrada u');
legend('Real', 'Simulado');
xlim([1 6])

subplot(3, 1, 2);
plot(tout, phi, 'b', 'LineWidth', 1.2); 
hold on;
plot(t_sim_phi, phi_sim, 'r--', 'LineWidth', 1.2);
hold off;
grid on;
xlabel('Tiempo (s)');
ylabel('\phi');
title('Ángulo \phi');
legend('Real', 'Simulado');
xlim([1 6])

subplot(3, 1, 3);
plot(tout, theta, 'b', 'LineWidth', 1.2); 
hold on;
plot(t_sim_theta, theta_sim, 'r--', 'LineWidth', 1.2);
hold off;
grid on;
xlabel('Tiempo (s)');
ylabel('\theta');
title('Ángulo \theta');
legend('Real', 'Simulado');
xlim([1 6])

%% PID
vars = out.who;
for i = 1:length(vars)
    assignin('base', vars{i}, out.(vars{i}));
end

%Matlab hizo cosas raras con los tiempos
t_sim_u     = linspace(0, 6, length(u_sim))';
t_sim_phi   = linspace(0, 6, length(phi_sim))';
t_sim_theta = linspace(0, 6, length(theta_sim))';

phi=phi-90;

figure;
subplot(3, 1, 1);
plot(tout, u, 'b', 'LineWidth', 1.2); 
hold on;
plot(t_sim_u, u_sim, 'r--', 'LineWidth', 1.2);
hold off;
grid on;
xlabel('Tiempo (s)');
ylabel('u');
title('Entrada u');
legend('Real', 'Simulado');
xlim([1 6])

subplot(3, 1, 2);
plot(tout, phi, 'b', 'LineWidth', 1.2); 
hold on;
plot(t_sim_phi, phi_sim, 'r--', 'LineWidth', 1.2);
hold off;
grid on;
xlabel('Tiempo (s)');
ylabel('\phi');
title('Ángulo \phi');
legend('Real', 'Simulado');
xlim([1 6])

subplot(3, 1, 3);
plot(tout, theta, 'b', 'LineWidth', 1.2); 
hold on;
plot(t_sim_theta, theta_sim, 'r--', 'LineWidth', 1.2);
hold off;
grid on;
xlabel('Tiempo (s)');
ylabel('\theta');
title('Ángulo \theta');
legend('Real', 'Simulado');
xlim([1 6])

