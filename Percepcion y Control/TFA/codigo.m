%% INICIALIZACIÓN DE ROS
close all;
clear all;
rosshutdown; % Cierra cualquier conexión previa de ROS
setenv('ROS_MASTER_URI','http://192.168.1.138:11311'); % IP del simulador STDR
setenv('ROS_ IP','192.168.1.49'); % IP local de tu máquina
rosinit % Inicia la conexión con el nodo maestro de ROS

%% DECLARACIÓN DE VARIABLES NECESARIAS PARA EL CONTROL
map_size_metros = 15;
celda_metros = 0.06;
n_celdas = round(map_size_metros / celda_metros);
Mapa = 0.5 * ones(n_celdas, n_celdas);
figure;
hMapa = imshow(Mapa, 'InitialMagnification', 'fit');
title('Mapa Explorado');
drawnow;

%% SUBSCRIPCIÓN A TOPICS DE ROS
odom = rossubscriber('/robot0/odom');
laser = rossubscriber('/robot0/laser_1');

%% PUBLICADOR DE VELOCIDAD
pub = rospublisher('/robot0/cmd_vel', 'geometry_msgs/Twist');
msg_vel = rosmessage(pub);

%% FRECUENCIA DEL BUCLE
r = robotics.Rate(10);
waitfor(r);

%% ESPERA PARA RECIBIR MENSAJES INICIALES
pause(3);
while (strcmp(odom.LatestMessage.ChildFrameId,'robot0') ~= 1)
    odom.LatestMessage
end

%% PARÁMETROS DE CONTROL
vel_lineal_max = 1.0;
vel_angular_max = 1.0;

%% INICIALIZACIÓN DEL TEMPORIZADOR
tiempo_inicio = rostime('now');
tiempo_maximo = 4 * 60;

%% DEFINICIÓN DE PUNTOS OBJETIVO
objetivos = [
    13.5, 1.5;
    13.5, 13.5;
    1.5, 13.5;
    6.0, 1.5;
    13.5, 7.5;
    6.0, 13.5;
    1.5, 7.5;
    7.5, 7.5
];
objetivo_idx = 1;
modo = "avanzar";
contador_esquiva = 0;
contador_objetivo = 0;
max_pasos_objetivo = 300; % Límite de pasos intentando alcanzar un objetivo

%% BUCLE PRINCIPAL
while true
    tiempo_actual = rostime('now');
    if (tiempo_actual.Sec - tiempo_inicio.Sec) > tiempo_maximo
        disp('Tiempo de exploración agotado. Guardando mapa...');
        break;
    end

    odomMsg = receive(odom, 1);
    laserMsg = receive(laser, 1);
    Mapa = actualizarMapa(Mapa, odomMsg, laserMsg, celda_metros, n_celdas);

    pos = odomMsg.Pose.Pose.Position;
    x_robot = pos.X;
    y_robot = pos.Y;

    if objetivo_idx > size(objetivos,1)
        objetivo = [7.5, 7.5];
    else
        objetivo = objetivos(objetivo_idx, :);
    end

    angulo_deseado = atan2(objetivo(2) - y_robot, objetivo(1) - x_robot);
    ori = odomMsg.Pose.Pose.Orientation;
    yaw = quat2eul([ori.W ori.X ori.Y ori.Z]);
    yaw = yaw(1);
    error_ang = atan2(sin(angulo_deseado - yaw), cos(angulo_deseado - yaw));
    dist = norm([objetivo(1) - x_robot, objetivo(2) - y_robot]);

    % Detección de obstáculos
    ranges = laserMsg.Ranges;
    n = length(ranges);
    idx_ini = round(n*0.1);
    idx_fin = round(n*0.9);
    zona_frontal = ranges([1:idx_ini, end-idx_fin:end]);
    obstaculo_cerca = min(zona_frontal) < 0.6;

    switch modo
        case "avanzar"
            contador_objetivo = contador_objetivo + 1;
            if dist > 0.3
                if obstaculo_cerca
                    modo = "esquivar";
                    contador_esquiva = 0;
                    continue;
                elseif abs(error_ang) > 0.2
                    vel_lineal = 0;
                    vel_angular = max(-vel_angular_max, min(vel_angular_max, error_ang)) * 0.3; % Giro más lento
                else
                    vel_lineal = vel_lineal_max * 0.5;
                    vel_angular = max(-vel_angular_max, min(vel_angular_max, error_ang)) * 0.2;
                end
            else
                objetivo_idx = objetivo_idx + 1;
                contador_objetivo = 0;
                vel_lineal = 0;
                vel_angular = 0;
            end
            if contador_objetivo > max_pasos_objetivo
                fprintf("Objetivo %d no alcanzado. Pasando al siguiente.\n", objetivo_idx);
                objetivo_idx = objetivo_idx + 1;
                contador_objetivo = 0;
                vel_lineal = 0;
                vel_angular = 0;
            end
        case "esquivar"
            contador_esquiva = contador_esquiva + 1;

            % Detección extendida del obstáculo más próximo en zona frontal amplia
            obstaculo_frontal = min(zona_frontal) < 0.2;

            % Estrategia de giro hacia el lado más libre
            derecha = min(ranges(1:round(n*0.4)));
            izquierda = min(ranges(round(n*0.6):end));
            if izquierda > derecha
                vel_angular = vel_angular_max * 0.15;
            else
                vel_angular = -vel_angular_max * 0.15;
            end

            % Avance lento solo si no hay obstáculo frontal inmediato
            if obstaculo_frontal
                vel_lineal = 0; % No avanzar si hay obstáculo justo delante
            else
                vel_lineal = vel_lineal_max * 0.05;
            end

            % Volver a avanzar si ha pasado tiempo suficiente sin obstáculos
            if contador_esquiva > 20 && ~obstaculo_frontal
                modo = "avanzar";
            end;
            if izquierda > derecha
                vel_angular = vel_angular_max * 0.1; % Giro aún más lento hacia la izquierda
            else
                vel_angular = -vel_angular_max * 0.1; % Giro aún más lento hacia la derecha
            end
            vel_lineal = vel_lineal_max * 0.05; % Avance muy lento para rodear el obstáculo
            if contador_esquiva > 15 && ~obstaculo_cerca
                modo = "avanzar";
            end;
            if izquierda > derecha
                vel_angular = vel_angular_max * 0.2;
            else
                vel_angular = -vel_angular_max * 0.2;
            end
            vel_lineal = vel_lineal_max * 0.1;
            if contador_esquiva > 10 && ~obstaculo_cerca
                modo = "avanzar";
            end
    end

    msg_vel.Linear.X = vel_lineal;
    msg_vel.Angular.Z = vel_angular;
    send(pub, msg_vel);

    set(hMapa, 'CData', Mapa);
    drawnow;
    waitfor(r);
end

%% GUARDADO FINAL
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
        x_ini = pos.X;
        y_ini = pos.Y;
        pasos = floor(distancia / (celda_metros / 2));
        for k = 1:pasos
            x = x_ini + (k * (celda_metros / 2)) * cos(angulo_global);
            y = y_ini + (k * (celda_metros / 2)) * sin(angulo_global);
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
