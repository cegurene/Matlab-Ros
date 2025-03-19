%% Funcion de calidad
function Q = calidad_paredes_sonar()
    odom=rossubscriber('/robot0/odom'); % Subscripción a la odometría
    sonar0=rossubscriber('/robot0/sonar_0'); % Subscripción al sonar delantero
    sonarIzq=rossubscriber('/robot0/sonar_1'); % Subscripción al sonar1
    sonarDer=rossubscriber('/robot0/sonar_2'); % Subscripción al sonar2
    sonarAtras1=rossubscriber('/robot0/sonar_3'); % Subscripción al sonar3
    sonarAtras2=rossubscriber('/robot0/sonar_4'); % Subscripción al sonar4
    pause(2)

    initpos=odom.LatestMessage.Pose.Pose.Position;

    % Estimación de la posición de las paredes
    y_front = initpos.Y + sonar0.LatestMessage.Range_;
    x_left = initpos.X - sonarIzq.LatestMessage.Range_;
    x_right = initpos.X + sonarDer.LatestMessage.Range_;
    
    % Cálculo de la pared trasera con dos sensores
    m_trasera = (sonarAtras1.LatestMessage.Range_ - sonarAtras2.LatestMessage.Range_) / (-2 * 0.3);
    b_trasera = (initpos.Y - sonarAtras1.LatestMessage.Range_) - m_trasera * (initpos.X + 0.3);

    % Estimación de pendientes
    pendiente_horizontales = [(y_front - b_trasera) / (2 * initpos.X)];
    pendiente_verticales = [(x_right - x_left) / (2 * initpos.Y)];
    
    % Variabilidad en las mediciones traseras
    sigma_trasera = abs(sonarAtras1.LatestMessage.Range_ - sonarAtras2.LatestMessage.Range_);
    
    % Desviaciones estándar
    sigma_horizontales = std(pendiente_horizontales);
    sigma_verticales = std(pendiente_verticales);
    
    % Función de calidad considerando la alineación de los dos sensores traseros
    Q = 1 - (sigma_horizontales + sigma_verticales + sigma_trasera) / 3;
    
    % Asegurar que Q esté en el rango [0,1]
    Q = max(0, min(1, Q));
end

%% Funcion paredes
function [codigo, confianza] = detectar_paredes_sonar()
    sonar0=rossubscriber('/robot0/sonar_0'); % Subscripción al sonar delantero
    sonarIzq=rossubscriber('/robot0/sonar_1'); % Subscripción al sonar1
    sonarDer=rossubscriber('/robot0/sonar_2'); % Subscripción al sonar2
    sonarAtras1=rossubscriber('/robot0/sonar_3'); % Subscripción al sonar3
    sonarAtras2=rossubscriber('/robot0/sonar_4'); % Subscripción al sonar4
    pause(2)

    % Definir umbral de detección de pared (por ejemplo, 2 metros)
    umbral = 2; 
    
    % Inicializar código binario de paredes
    codigo = [0 0 0 0];
    
    % Verificar detección de paredes
    if sonar0.LatestMessage.Range_ < umbral
        codigo(1) = 1; % Bit más significativo para la pared frontal
    end
    if sonarAtras1.LatestMessage.Range_ < umbral & sonarAtras2.LatestMessage.Range_ < umbral
        codigo(3) = 1; % Pared trasera
    end
    if sonarIzq.LatestMessage.Range_ < umbral
        codigo(2) = 1; % Pared izquierda
    end
    if sonarDer.LatestMessage.Range_ < umbral
        codigo(4) = 1; % Pared derecha
    end
    
    % Calcular confianza basada en perpendicularidad de las paredes detectadas
    confianza = calidad_paredes_sonar();
end

%% Ejemplo
[codigo, confianza] = detectar_paredes_sonar();
codigo = sprintf('%d%d%d%d', codigo);
% 1er digito: pared delantera
% 2º digito: pared izquierda
% 3º digito: pared trasera
% 4º digito: pared derecha
% 1: ocupado, 0: libre

disp(['Código de paredes detectadas: ', num2str(codigo)]);
disp(['Grado de confianza: ', num2str(confianza)]);