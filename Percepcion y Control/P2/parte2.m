%% INICIALIZACIÓN DE ROS (COMPLETAR ESPACIOS CON LAS DIRECCIONES IP)
close all;
clear all;
setenv('ROS_MASTER_URI','http://192.168.1.125:11311'); %IP del simulador
setenv('ROS_ IP','192.168.1.49'); %Mi IP
rosinit % Inicialización de ROS en la IP correspondiente

%% DECLARACIÓN DE VARIABLES NECESARIAS PARA EL CONTROL
MAX_TIME = 1000;  % Numero máximo de iteraciones
medidas = zeros(5,1000);

D = 3;          % NUEVO: Distancia deseada a la pared (metros)
K_ori = 0.5;      % NUEVO: Ganancia para error de orientación
K_dist = 0.5;     % NUEVO: Ganancia para error lateral

%% DECLARACIÓN DE SUBSCRIBERS
odom = rossubscriber('/robot0/odom'); % Subscripcion a la odometria
sonar0 = rossubscriber('/robot0/sonar_0', rostype.sensor_msgs_Range);

%% DECLARACIÓN DE PUBLISHERS
pub = rospublisher('/robot0/cmd_vel', 'geometry_msgs/Twist'); %
msg_vel=rosmessage(pub);  % Creamos un mensaje del tipo declarado en "pub" (geometry_msgs/Twist)
msg_sonar0 = rosmessage(sonar0);

%% Definimos la periodicidad del bucle (10hz)
r = robotics.Rate(10);
waitfor(r);

%% Esperamos entre 2 y 5 segundos antes de leer el primer mensaje para aseguramos que empiezan a llegar mensajes.
pause(3);

%% Nos aseguramos recibir un mensaje relacionado con el robot
while (strcmp(odom.LatestMessage.ChildFrameId,'robot0')~=1)
    odom.LatestMessage
end

%% Inicializamos variables para el control
i = 1;
pos = [0, 0, 0];
dist = 0;
lastpos = odom.LatestMessage.Pose.Pose.Position;  % Posicion inicial
lastdist = 0;
lastdistav = 0;

%% Bucle de control
while (i < 1001)
    %i = i + 1;

    %% Obtenemos la posición y medidas de sonar
    pos=odom.LatestMessage.Pose.Pose.Position;
    msg_sonar0 = receive(sonar0);

    %% Calculamos la distancia avanzada y medimos la distancia a la pared
    dx = pos.X - lastpos.X;  % NUEVO
    dy = pos.Y - lastpos.Y;  % NUEVO
    distav = sqrt(dx^2 + dy^2); % NUEVO: avance lineal

    dist = double(msg_sonar0.Range_) + 0.125;
    if (dist > 5)
        dist = 5;
    end

    %% Calculamos el error de distancia y orientación
    if distav > 0.001  % NUEVO: evitar división por cero
        Eori = atan((dist - lastdist) / distav);
    else
        Eori = 0;
    end

    Edist = dist - D;
    medidas(1,i) = dist;
    medidas(2,i) = lastdist;  % valor anterior de distancia
    medidas(3,i) = distav;
    medidas(4,i) = Eori;
    medidas(5,i) = Edist;

    %% Calculamos las consignas de velocidades
    consigna_vel_linear = 0.3;
    consigna_vel_ang = K_ori * Eori + K_dist * Edist;
    consigna_vel_ang = max(-0.5, min(consigna_vel_ang, 0.5));

    %% Condición de parada
    if i > 75
        if (abs(Edist) < 0.01) && (abs(Eori) < 0.01)
            fprintf('Parada\n');
            break
        end
    end

    %% Aplicamos consignas de control
    msg_vel.Linear.X = consigna_vel_linear;
    msg_vel.Linear.Y = 0;
    msg_vel.Linear.Z = 0;
    msg_vel.Angular.X = 0;
    msg_vel.Angular.Y = 0;
    msg_vel.Angular.Z = consigna_vel_ang;

    % Comando de velocidad
    send(pub,msg_vel);

    lastpos = pos;
    lastdist = dist;
    lastvAng = msg_vel.Angular.Z;
    lastdistav = distav;

    % Temporización del bucle según el parámetro establecido en r
    waitfor(r);

    % if (i == MAX_TIME)
    %     break
    % end
    i = i + 1;
end

save('medidas.mat', 'medidas');

%% DESCONEXIÓN DE ROS
rosshutdown;