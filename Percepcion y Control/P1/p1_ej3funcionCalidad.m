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

    calidad = 1 - (error1 + error2) / 2;  % Calidad en rango [0,1]
end

%% Ejemplo
Q = calidad_paredes_laser();