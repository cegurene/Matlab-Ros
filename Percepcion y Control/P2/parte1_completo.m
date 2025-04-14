%% INICIALIZACIÓN DE ROS
close all;
clear all;
setenv('ROS_MASTER_URI','http://192.168.1.125:11311'); %IP del simulador
setenv('ROS_ IP','192.168.1.49'); %Mi IP
rosinit % Inicialización de ROS en la IP correspondiente

%% DECLARACIÓN DE VARIABLES NECESARIAS PARA EL CONTROL
% Punto objetivo (puedes cambiar esto por una entrada del usuario)
destinos = [ 15, 5;
              5, 15;
             10, 10;
             15, 15]; 

num_destinos = size(destinos, 1);

% Ganancias del controlador P
Kp_dist = 0.5;
Kp_ang = 1.0;

%% DECLARACIÓN DE SUBSCRIBERS
odom = rossubscriber('/robot0/odom'); % Subscripción a la odometría

%% DECLARACIÓN DE PUBLISHERS
pub = rospublisher('/robot0/cmd_vel', 'geometry_msgs/Twist'); %
msg_vel=rosmessage(pub); % Creamos un mensaje del tipo declarado en "pub" (geometry_msgs/Twist)

%% Definimos la perodicidad del bucle (10 hz)
r = robotics.Rate(10);
waitfor(r);

%% Esperamos entre 2 y 5 segundos antes de leer el primer mensaje para aseguramos que empiezan a llegar mensajes.
pause(3);

%% Nos aseguramos recibir un mensaje relacionado con el robot
while (strcmp(odom.LatestMessage.ChildFrameId,'robot0')~=1)
    odom.LatestMessage
end

%% Umbrales para condiciones de parada del robot
umbral_distancia = 0.1;       % Umbral para detenerse (metros)
umbral_angulo = deg2rad(5);   % Umbral angular para detenerse (radianes)

%% Funcion para corregir el error de orientacion
function E_ori_corregido = fcn(E_ori)
    if E_ori <-pi
        E_ori = E_ori + (2*pi)
    end
    if E_ori > pi
        E_ori = E_ori - (2*pi)
    end
end

%% Bucle de control
for i_dest = 1:num_destinos
    x_destino = destinos(i_dest, 1);
    y_destino = destinos(i_dest, 2);
    fprintf('\nMoviéndose al destino %d: (%.2f, %.2f)\n', i_dest, x_destino, y_destino);

    while true
        % Obtenemos posicion y orientacion actuales
        pos = odom.LatestMessage.Pose.Pose.Position;
        ori = odom.LatestMessage.Pose.Pose.Orientation;
        yaw = quat2eul([ori.W ori.X ori.Y ori.Z]);
        yaw = yaw(1);  % Solo eje Z (ángulo yaw)

        % calculamos errores
        dx = x_destino - pos.X;
        dy = y_destino - pos.Y;
        Edist = sqrt(dx^2 + dy^2);

        ang_deseado = atan2(dy, dx);
        Eori = atan2(sin(ang_deseado - yaw), cos(ang_deseado - yaw)); % Normalizado

        % Calculamos las consignas de velocidades
        consigna_vel_linear = Kp_dist * Edist;
        consigna_vel_ang = Kp_ang * Eori;

        % Saturación
        consigna_vel_linear = min(consigna_vel_linear, 1.0);
        consigna_vel_ang = min(max(consigna_vel_ang, -0.5), 0.5);

        % Condicion de parada
        if (Edist < umbral_distancia) && (abs(Eori) < umbral_angulo)
            break;
        end

        % Aplicamos consignas de control
        msg_vel.Linear.X = consigna_vel_linear;
        msg_vel.Linear.Y = 0;
        msg_vel.Linear.Z = 0;
        msg_vel.Angular.X = 0;
        msg_vel.Angular.Y = 0;
        msg_vel.Angular.Z = consigna_vel_ang;
        send(pub, msg_vel);

        waitfor(r);
    end

    % Detener al robot
    msg_vel.Linear.X = 0;
    msg_vel.Angular.Z = 0;
    send(pub, msg_vel);

    % Mostramos por pantalla la posicion
    fprintf('Posición actual: X = %.2f, Y = %.2f\n', pos.X, pos.Y);

    pause(1);  % Espera antes de moverse al siguiente punto
end

%% FINALIZACIÓN
fprintf('\nTodos los destinos han sido alcanzados.\n');
rosshutdown;