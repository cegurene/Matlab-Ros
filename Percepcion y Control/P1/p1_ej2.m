%% Grafica 1000 medidas

% Número de muestras a capturar
num_muestras = 1000;

% Inicializar un array para almacenar los valores del sonar
sonar_data = zeros(1, num_muestras);

% Capturar los datos
for i = 1:num_muestras
    msg = sonar0.LatestMessage; % Espera hasta 10 segundos por un mensaje
    sonar_data(i) = msg.Range_; % Capturar distancia
    % Temporización del bucle según el parámetro establecido en r
    waitfor(r);
end

plot(1:num_muestras, sonar_data, 'b-', 'LineWidth', 1.5);
xlabel('Muestra');
ylabel('Distancia del sonar (m)');
title('Mediciones del sonar');
grid on;

%% Media movil

% Número de muestras a capturar
num_muestras = 1000;

% Inicializar un array para almacenar los valores del sonar
sonar_data = zeros(1, num_muestras);
sonar_filtrado = zeros(1, num_muestras); % Datos con filtro

% Capturar los datos
for i = 1:num_muestras
    msg = sonar0.LatestMessage; % Espera hasta 10 segundos por mensaje
    sonar_data(i) = msg.Range_; % Capturar la distancia medida
    
    % Aplicar filtro de media móvil (últimos 5 valores)
    if i >= 5
        sonar_filtrado(i) = mean(sonar_data(i-4:i)); % Promedio de los últimos 5 valores
    else
        sonar_filtrado(i) = mean(sonar_data(1:i)); % Promedio de las muestras disponibles
    end
end

% Graficar los datos originales y filtrados
figure;
plot(1:num_muestras, sonar_data, 'b-', 'LineWidth', 1, 'DisplayName', 'Datos originales');
hold on;
plot(1:num_muestras, sonar_filtrado, 'r-', 'LineWidth', 2, 'DisplayName', 'Filtro media móvil (5)');
hold off;
xlabel('Muestra');
ylabel('Distancia del sonar (m)');
title('Mediciones del sonar con filtro de media móvil');
legend();
grid on;

rosshutdown;