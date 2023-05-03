clear all
close all
clc

% Definir el tiempo de muestreo
dt = 0.01; % segundos

% Definir la duración de la señal de entrada
dur = 10; % segundos

% Definimos el vector de tiempo
t = [0:dt:dur-dt];

RH = .2;
RL = RH;

load('DatosExamen.mat')

ylpf = zeros(size(x));
yhpf = zeros(size(x));
y = zeros(size(x));

%% Aquí implementamos el filtro
for i = 1:length(x)

    %-------------------------------
     % Filtro de baja frecuencia
    if i>1
        yhpf(i) = (dt/(RL+dt))*x_hf(i) + (RL/(RL+dt))*yhpf(i-1);
    else
        yhpf(i) = x_hf(i);
    end
        
    % Filtro de alta frecuencia
    if i>1
        ylpf(i) = (RH/(RH+dt))*ylpf(i-1) + (RH/(RH+dt))* (x_lf(i) - x_lf(i-1));
    else
        ylpf(i) = x_lf(i);
    end

    % Sumamos la salida de los filtros
    y(i) = yhpf(i) + ylpf(i);
    
end

% Graficar las señales de entrada y salida
plot(t,x,'color','b');
hold on
 plot(t,x_hf,'color','r');
plot(t,x_lf,'color','y');
plot(t,y,'color','k');
 legend('Señal de entrada sin ruido','Señal de entrada con ruido de alta frecuencia', ...
     'Señal de entrada con ruido de baja frecuencia', 'Señal de salida');
xlabel('Tiempo (s)');
ylabel('Amplitud');
