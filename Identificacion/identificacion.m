clear ; close all; clc
s = tf('s');

%Variables
cantidad_muestras = 3100;
Ts = 0.02;
datos = zeros(cantidad_muestras, 3);
muestras_leidas = 0;

%Configurar Puerto Serie
puerto = "COM8";
baudios = 115200;

s = serialport(puerto, baudios);
flush(s); % Limpiar el buffer

% Fase de sincronización: buscar el primer 'abcd'
disp('Sincronizando...');
while true
    if read(s, 1, "uint8") == 97
        if read(s, 1, "uint8") == 98 && read(s, 1, "uint8") == 99 && read(s, 1, "uint8") == 100
            break; % Sincronización exitosa
        end
    end
end
disp('Sincronizado. Leyendo datos...');

while muestras_leidas < cantidad_muestras
    muestras_leidas = muestras_leidas + 1;
    datos(muestras_leidas, :) = read(s, 3, "single");
    % Leer el siguiente encabezado para preparar el próximo ciclo
    if muestras_leidas < cantidad_muestras
        hdr = read(s, 4, "uint8");
        if ~isequal(hdr, [97, 98, 99, 100])
            disp('Se perdió la sincronización. Deteniendo lectura.');
            break;
        end
    end
end

% Cerrar puerto
clear s;

% Separar las variables
dato1 = datos(:, 1); % Ángulo de referencia
dato2 = datos(:, 2); % Ángulo pote
dato3 = datos(:, 3); % Ángulo imu

%% Transferencia completa (de u a theta) de orden 4
close all;

u = double(dato1(:));
y = double(dato3(:));

% Suponer que las primeras muestras corresponden al equilibrio
u = u - 90;
y = y - 0;

A = [ ...
    y(4:end-1), ...  % y(k-1)
    y(3:end-2), ...  % y(k-2)
    y(2:end-3), ...  % y(k-3)
    y(1:end-4), ...  % y(k-4)
    u(3:end-2), ...  % u(k-2)
    u(2:end-3)  ...  % u(k-3)
    u(1:end-4)  ...  % u(k-4)
];
b = y(5:end);        % y(k)

Xhat = A \ b;

bc = A*Xhat;
% Error cuadratico
e2 = (b-bc).^2;
error_cuadratico = sum(e2);
RMSE = sqrt(mean(e2));

% Mostrar resultados
disp('Coeficientes estimados Xhat:');
disp(Xhat);

fprintf('Error cuadratico total: %.6f\n', error_cuadratico);
fprintf('RMSE: %.6f\n', RMSE);

% Gráficos
tiempo = (0:length(b)-1) * Ts;
tiempo2 = (0:length(dato1)-1) * Ts;
figure;
subplot(2, 1, 1);
plot(tiempo, b, 'b', LineWidth=2);
hold on;
plot(tiempo, bc, 'r--', LineWidth=2);
grid on;
ylabel('Ángulo salida Tita');
legend('Ángulo medido', 'Ángulo estimado');
title('Identificacion por cuadrados minimos');
subplot(2, 1, 2);
plot(tiempo2, dato1, "b", LineWidth=2);
hold on;
plot(tiempo2, dato2, "g", LineWidth=2);
grid on;
xlabel('Tiempo (s)');
ylabel('Ángulo Servo Phi');
legend('Orden enviada', 'Orden ejecutada');

num = [0, 0, Xhat(5), Xhat(6), Xhat(7)]; 
den = [1, -Xhat(1), -Xhat(2), -Xhat(3), -Xhat(4)];

sys_d = tf(num, den, Ts);

figure;
pzmap(sys_d);
zgrid; % Agrega la grilla del plano Z
title('Mapa de Polos y Ceros del Sistema phi a theta Discreto Identificado');

figure;
bode(sys_d);
grid;
title("Bode Discreto de u a theta");

% Transferencia

sys_c = d2c(sys_d, "tustin")

figure;
pzmap(sys_c);
sgrid;
title('Mapa de Polos y Ceros del Sistema u a theta Continuo Identificado');

figure;
bode(sys_c);
grid;
title("Bode Continuo de u a theta");

%%  Simulación

[z_z, p_z, k_z] = zpkdata(sys_d, 'v');

p_z = [...
    0.9723 + 0.1882i, ...
    0.9723 - 0.1882i, ...
    0.4946 + 0.0000i, ...
    0.4946 + 0.0000i, ...
];

z_z = [6.0505, 1];

% Crear el objeto de función de transferencia
sys_ideal = zpk(z_z, p_z, k_z, Ts);

u = double(dato1(:));
y = double(dato3(:));

u = u - 90;
y = y - 0;

t = (0:length(y)-1)' * Ts;

y_sim = lsim(sys_d, u, t);
y_ideal = lsim(sys_ideal, u, t);


figure;
plot(t, y, 'b', 'LineWidth', 2);
hold on;
plot(t, y_sim, 'r--', 'LineWidth', 2);
plot(t, y_ideal, 'g--', 'LineWidth', 2);
grid on;
xlabel('Tiempo (s)');
ylabel('Ángulo del péndulo');
legend('Medido', 'Identificado', 'Correjido');
title('Validación de la transferencia completa');

figure;
pzmap(sys_ideal);
zgrid; % Agrega la grilla del plano Z
title('Mapa de Polos y Ceros del Sistema u a theta Discreto Correjido');

figure;
bode(sys_ideal);
grid;
title("Bode Discreto de u a theta");

sys_corr = minreal(d2c(sys_ideal, "Matched"))

figure;
pzmap(sys_corr);
sgrid;
title('Mapa de Polos y Ceros del Sistema u a theta Continuo correjido');

figure;
bode(sys_corr);
grid;
title("Bode Continuo del sistema corregido");

%% Simulación del sistema continuo corregido
y_cont = lsim(sys_corr, u, t);

figure;
plot(t, y, 'b', 'LineWidth', 2);
hold on;
plot(t, y_cont, 'r--', 'LineWidth', 2);
grid on;
xlabel('Tiempo (s)');
ylabel('Ángulo del péndulo');
legend('Medido', 'Sistema Continuo Corregido');
title('Validación de la transferencia completa');
