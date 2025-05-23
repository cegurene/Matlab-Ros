%% INICIALIZACIÓN DE ROS
close all;
clear all;
rosshutdown; % Cierra cualquier conexión previa de ROS
setenv('ROS_MASTER_URI','http://192.168.1.146:11311'); % IP del simulador STDR
setenv('ROS_ IP','192.168.1.49'); % IP local de tu máquina
rosinit % Inicia la conexión con el nodo maestro de ROS

%% SUBSCRIPCIÓN A TOPICS DE ROS
odom = rossubscriber('/robot0/odom');
laser = rossubscriber('/robot0/laser_1');
sonarDel_Izq = rossubscriber('/robot0/sonar_2'); % Frontal
sonarDel_Der = rossubscriber('/robot0/sonar_3'); % Frontal
sonarIzq = rossubscriber('/robot0/sonar_0'); % Izquierda
sonarDer = rossubscriber('/robot0/sonar_5'); % Derecha

msg_vel = rosmessage('geometry_msgs/Twist');
pub = rospublisher('/robot0/cmd_vel', 'geometry_msgs/Twist');

%% FRECUENCIA DEL BUCLE
r = robotics.Rate(10); % 10 Hz
waitfor(r);

%% ESPERA PARA RECIBIR MENSAJES INICIALES
pause(3);
while (strcmp(odom.LatestMessage.ChildFrameId,'robot0') ~= 1)
    odom.LatestMessage
end

%% DECLARACIÓN DE VARIABLES NECESARIAS PARA EL CONTROL DEL MAPA
map_size_metros = 15;
celda_metros = 0.06;
n_celdas = round(map_size_metros / celda_metros);
Mapa = 0.5 * ones(n_celdas, n_celdas); % Inicializa el mapa con valor 0.5 (desconocido)
figure;
hMapa = imshow(Mapa, 'InitialMagnification', 'fit');
title('Mapa Explorado');
drawnow;

trayectoria_x = zeros(1, 300);
trayectoria_y = zeros(1, 300);
i = 1;

%% Variables de movimiento
D = 0.5;
K_ori = 2.0;
K_dist = 0.9;
vel_lineal = 0;
vel_angular = 0;

%% INICIALIZACIÓN DEL TEMPORIZADOR
last_dist_der = NaN; % Variable para almacenar la última distancia del sonar derecho
tiempo_inicio = rostime('now');
tiempo_maximo = 4 * 60; % 4 minutos

%% BUCLE PRINCIPAL
while true
    % Comprobación del tiempo de exploración
    tiempo_actual = rostime('now');
    if (tiempo_actual.Sec - tiempo_inicio.Sec) > tiempo_maximo
        disp('Tiempo de exploración agotado. Guardando mapa...');
        break;
    end

    % Recepción de datos de sensores
    odomMsg = receive(odom, 1);
    laserMsg = receive(laser, 1);
    msg_sonarDel_Izq = receive(sonarDel_Izq);
    msg_sonarDel_Der = receive(sonarDel_Der);
    msg_sonarIzq = receive(sonarIzq);
    msg_sonarDer = receive(sonarDer);

    % Actualización del mapa con las nuevas mediciones del LIDAR
    Mapa = actualizarMapa(Mapa, odomMsg, laserMsg, celda_metros, n_celdas);

    % Posición actual del robot
    pos = odomMsg.Pose.Pose.Position;
    x_robot = pos.X;
    y_robot = pos.Y;

    % Guardamos la posicion para mostrar la trayectoria
    if i > length(trayectoria_x)
        trayectoria_x(end+100) = 0;  % Aumenta el tamaño en bloques
        trayectoria_y(end+100) = 0;
    end
    trayectoria_x(i) = pos.X;
    trayectoria_y(i) = pos.Y;

    %% DETECCIÓN DE PAREDES
    dist_der = double(msg_sonarDer.Range_);
    dist_izq = double(msg_sonarIzq.Range_);
    dist_front_izq = double(msg_sonarDel_Izq.Range_);
    dist_front_der = double(msg_sonarDel_Der.Range_);

    %% LÓGICA DE MOVIMIENTO - SEGUIMIENTO DE PARED DERECHA
    dx = pos.X - x_robot;
    dy = pos.Y - y_robot;
    dist_av = sqrt(dx^2 + dy^2);

    % Seguimiento de pared derecha
    if isnan(last_dist_der)
        last_dist_der = dist_der;
    end

    % Calculo de errores
    if dist_av > 0.001
        Eori = atan((dist_der - last_dist_der) / dist_av);
    else
        Eori = 0;
    end

    Edist = dist_der - D;

    % Movimiento

    if dist_izq < 0.25  % Obstaculo izquierda
        vel_angular = -0.3;
    else
        if dist_front_izq<0.55 || dist_front_der<0.55  % Obstaculo delante
            vel_lineal = 0;
            vel_angular = 0.4;
        else
            % Obstaculo derecha
            vel_angular = -(K_ori * Eori + K_dist * Edist);
            vel_lineal = 0.5;
        end
    end
    

    last_dist_der = dist_der;

    % Enviamos velocidad y actualizamos mapa
    
    msg_vel.Linear.X = vel_lineal;
    msg_vel.Angular.Z = vel_angular;
    send(pub, msg_vel);

    set(hMapa, 'CData', Mapa);
    hold on;
    x_px = floor(trayectoria_x(1:i) / celda_metros) + 1;
    y_px = n_celdas - floor(trayectoria_y(1:i) / celda_metros);
    plot(x_px, y_px, 'r.', 'MarkerSize', 8); % trayectoria en rojo
    hold off;

    drawnow;
    i = i + 1;
    waitfor(r);
end

%% GUARDADO FINAL
msg_vel.Linear.X = 0;
msg_vel.Angular.Z = 0;
send(pub, msg_vel);

trayectoria_x = trayectoria_x(1:i-1);
trayectoria_y = trayectoria_y(1:i-1);

imwrite(Mapa, 'Mapa_Final.png');
rosshutdown;

%% FUNCIONES AUXILIARES
function Mapa = actualizarMapa(Mapa, odomMsg, laserMsg, celda_metros, n_celdas)
    pos = odomMsg.Pose.Pose.Position;
    ori = odomMsg.Pose.Pose.Orientation;
    yaw = quat2eul([ori.W ori.X ori.Y ori.Z]);
    yaw = yaw(1);

    num_rays = length(laserMsg.Ranges);
    angle_min = laserMsg.AngleMin;
    angle_increment = laserMsg.AngleIncrement;

    for i = 1:num_rays
        distancia = laserMsg.Ranges(i);
        if distancia < laserMsg.RangeMin || distancia > laserMsg.RangeMax
            continue;
        end

        angulo_global = yaw + angle_min + (i-1) * angle_increment;
        x_obs = pos.X + distancia * cos(angulo_global);
        y_obs = pos.Y + distancia * sin(angulo_global);

        pasos = floor(distancia / (celda_metros / 2));
        for k = 1:pasos
            x = pos.X + (k * (celda_metros / 2)) * cos(angulo_global);
            y = pos.Y + (k * (celda_metros / 2)) * sin(angulo_global);
            [fila, columna] = coordenadasAMapa(x, y, celda_metros, n_celdas);
            if fila >=1 && fila <= n_celdas && columna >=1 && columna <= n_celdas
                Mapa(fila, columna) = 1;
            end
        end

        [fila, columna] = coordenadasAMapa(x_obs, y_obs, celda_metros, n_celdas);
        if fila >=1 && fila <= n_celdas && columna >=1 && columna <= n_celdas
            Mapa(fila, columna) = 0;
        end
    end
end

function [fila, columna] = coordenadasAMapa(x, y, celda_metros, n_celdas)
    columna = floor(x / celda_metros) + 1;
    fila = n_celdas - floor(y / celda_metros);
end
