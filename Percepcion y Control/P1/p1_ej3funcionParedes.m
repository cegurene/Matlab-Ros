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
    %confianza = calidad_paredes_doble();
    confianza = 1;
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
