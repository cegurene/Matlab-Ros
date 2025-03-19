%% Funcion de calidad
function Q = calidad_paredes_laser()
    odom=rossubscriber('/robot0/odom'); % Subscripción a la odometría
    laser=rossubscriber('/robot0/laser_0'); % Subscripción al laser
    pause(2)

    n = length(laser.LatestMessage.Ranges);
    % Definir los índices de cada tercio
    tercio = floor(n / 3);
    idx1 = 1:tercio;
    idx2 = tercio+1:2*tercio;
    idx3 = 2*tercio+1:n;  % Incluye los elementos restantes en el último tercio

    initpos=odom.LatestMessage.Pose.Pose.Position;

    % Estimación de la posición de las paredes
    y_front = initpos.Y + mean(laser.LatestMessage.Ranges(idx2));
    x_left = initpos.X - mean(laser.LatestMessage.Ranges(idx3));
    x_right = initpos.X + mean(laser.LatestMessage.Ranges(idx1));

    % Ajustar rectas mediante mínimos cuadrados
    coef_derecha = polyfit(x_right(:,1), x_right(:,2), 1);
    coef_delantera = polyfit(y_front(:,1), y_front(:,2), 1);
    coef_izquierda = polyfit(x_left(:,1), x_left(:,2), 1);

    % Obtener pendientes de las rectas
    m1 = coef_derecha(1);
    m2 = coef_delantera(1);
    m3 = coef_izquierda(1);

    % Evaluar calidad: los productos de pendientes deben ser cercanos a -1 (perpendicularidad)
    error1 = abs(m1 * m2 + 1);
    error2 = abs(m2 * m3 + 1);

    Q = 1 - (error1 + error2) / 2;  % Calidad en rango [0,1]
end

%% Funcion
function [codigo, confianza] = detectar_paredes_laser()
    laser=rossubscriber('/robot0/laser_0'); % Subscripción al laser
    pause(2)

    n = length(laser.LatestMessage.Ranges);
    % Definir los índices de cada tercio
    tercio = floor(n / 3);
    idx1 = 1:tercio;
    idx2 = tercio+1:2*tercio;
    idx3 = 2*tercio+1:n;  % Incluye los elementos restantes en el último tercio

    % Definir umbral de detección de pared (por ejemplo, 2 metros)
    umbral = 2; 
    
    % Inicializar código binario de paredes
    codigo = [0 0 0];
    
    % Verificar detección de paredes
    if mean(laser.LatestMessage.Ranges(idx2)) < umbral
        codigo(1) = 1; % Bit más significativo para la pared frontal
    end
    if mean(laser.LatestMessage.Ranges(idx3)) < umbral
        codigo(2) = 1; % Pared izquierda
    end
    if mean(laser.LatestMessage.Ranges(idx1)) < umbral
        codigo(3) = 1; % Pared derecha
    end
    
    % Calcular confianza basada en perpendicularidad de las paredes detectadas
    confianza = calidad_paredes_laser();
end

%% Ejemplo
% Llamada a la función
[codigo, confianza] = detectar_paredes_laser();

codigo = sprintf('%d%d%d%d', codigo);
% 1er digito: pared delantera
% 2º digito: pared izquierda
% 3º digito: pared derecha

% Mostrar resultados
disp(['Código de paredes detectadas: ', num2str(codigo)]);
disp(['Grado de confianza: ', num2str(confianza)]);