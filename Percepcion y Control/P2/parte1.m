%% INICIALIZACIÓN DE ROS
close all;
clear all;
setenv('ROS_MASTER_URI','http://172.29.29.108:11311'); %IP del simulador
setenv('ROS_ IP','172.29.29.112'); %Mi IP
rosinit % Inicialización de ROS en la IP correspondiente

%% DECLARACIÓN DE VARIABLES NECESARIAS PARA EL CONTROL
% DECLARACIÓN DE SUBSCRIBERS
odom = rossubscriber('/robot0/odom'); % Subscripción a la odometría

% DECLARACIÓN DE PUBLISHERS
pub = rospublisher('/robot0/cmd_vel', 'geometry_msgs/Twist'); %
msg_vel=rosmessage(pub); % Creamos un mensaje del tipo declarado en "pub" (geometry_msgs/Twist)

% Definimos la perodicidad del bucle (10 hz)
r = robotics.Rate(10);
waitfor(r);

% Esperamos entre 2 y 5 segundos antes de leer el primer mensaje para aseguramos que empiezan a llegar mensajes.
pause(3);

% Nos aseguramos recibir un mensaje relacionado con el robot
while (strcmp(odom.LatestMessage.ChildFrameId,'robot0')~=1)
    odom.LatestMessage
end

%% Umbrales para condiciones de parada del robot
umbral_distancia = ;
umbral_angulo = ;


%% Funcion de desplazamiento
function desplazar(x, y, o)

end

%% Bucle de control infinito
while (1)
    % Obtenemos la posición y orientación actuales
    pos=odom.LatestMessage.Pose.Pose.Position;
    ori=odom.LatestMessage.Pose.Pose.Orientation;
    yaw=quat2eul([ori.W ori.X ori.Y ori.Z]);
    yaw=yaw(1);

    % Calculamos el error de distancia
    Edist = ;

    % Calculamos el error de orientación
    Eori = ;

    % Calculamos las consignas de velocidades
    consigna_vel_linear = ;
    consigna_vel_ang = ;

    % Condición de parada
    if (Edist<umbral_distancia) && (abs(Eori)<umbral_angulo)
        break;
    end

    % Aplicamos consignas de control
    msg_vel.Linear.X = consigna_vel_linear;
    msg_vel.Linear.Y = 0;
    msg_vel.Linear.Z = 0;
    msg_vel.Angular.X = 0;
    msg_vel.Angular.Y = 0;
    msg_vel.Angular.Z = consigna_vel_ang;

    % Comando de velocidad
    send(pub,msg_vel);

    % Temporización del bucle según el parámetro establecido en r
    waitfor(r);
end
