%% Grafica 1000 medidas

% Número de medidas a capturar
num_muestras = 1000;

% Inicializar un array para almacenar los valores del laser
laser_data = zeros(1, num_muestras);

% Capturar los datos
for i = 1:num_muestras
    msg = laser.LatestMessage; % Espera hasta 10 segundos por un mensaje
    laser_data(i) = mean(msg.Ranges); % Capturar distancia
    % Temporización del bucle según el parámetro establecido en r
    waitfor(r);
end


% Graficar las medidas originales
plot(1:num_muestras, laser_data, 'b-', 'LineWidth', 1.5);
xlabel('Muestra');
ylabel('Distancia del laser (m)');
title('Mediciones del laser');
grid on;

%% Media movil

% Número de muestras a capturar
num_muestras = 1000;

% Inicializar un array para almacenar los valores del laser
laser_data = zeros(1, num_muestras);
laser_filtrado = zeros(1, num_muestras); % Datos con filtro

% Capturar los datos
for i = 1:num_muestras
    msg = laser.LatestMessage; % Espera hasta 10 segundos por mensaje
    laser_data(i) = mean(msg.Ranges); % Capturar la distancia medida
    
    % Aplicar filtro de media móvil (últimos 5 valores)
    if i >= 5
        laser_filtrado(i) = mean(laser_data(i-4:i)); % Promedio de los últimos 5 valores
    else
        laser_filtrado(i) = mean(laser_data(1:i)); % Promedio de las muestras disponibles
    end
end

% Graficar los datos originales y filtrados
figure;
plot(1:num_muestras, laser_data, 'b-', 'LineWidth', 1, 'DisplayName', 'Datos originales');
hold on;
plot(1:num_muestras, laser_filtrado, 'r-', 'LineWidth', 2, 'DisplayName', 'Filtro media móvil (5)');
hold off;
xlabel('Muestra');
ylabel('Distancia del laser (m)');
title('Mediciones del laser con filtro de media móvil');
legend();
grid on;

rosshutdown;