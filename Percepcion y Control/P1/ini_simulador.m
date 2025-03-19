%% INICIALIZACIÓN DE ROS
% Se definen las variables de entorno ROS_MASTER_URI (ip del Master) y ROS_IP (IP
% de la máquina donde se ejecuta Matlab). Si se está conectado a la misma red, la
% variable ROS_IP no es necesario definirla.
clear all;
close all;
rosshutdown;
setenv('ROS_MASTER_URI','http://172.29.29.107:11311') %IP del simulador
setenv('ROS_IP','172.29.30.63') %Mi IP
rosinit % Inicialización de ROS

%% DECLARACIÓN DE SUBSCRIBERS
odom=rossubscriber('/robot0/odom'); % Subscripción a la odometría
laser=rossubscriber('/robot0/laser_0'); % Subscripción al laser
sonar0=rossubscriber('/robot0/sonar_0'); % Subscripción al sonar delantero
sonarIzq=rossubscriber('/robot0/sonar_1'); % Subscripción al sonar1
sonarDer=rossubscriber('/robot0/sonar_2'); % Subscripción al sonar2
sonarAtras1=rossubscriber('/robot0/sonar_3'); % Subscripción al sonar3
sonarAtras2=rossubscriber('/robot0/sonar_4'); % Subscripción al sonar4

pause(2)

%% DECLARACIÓN DE PUBLISHERS
pub = rospublisher('/robot0/cmd_vel', 'geometry_msgs/Twist');


%% Definimos la perodicidad del bucle (10 hz)
r = robotics.Rate(10);

%% Nos aseguramos recibir un mensaje relacionado con el robot "robot0"
pause(1); % Esperamos 1 segundo para asegurarnos que ha llegado algún mensaje odom,
% porque sino ls función strcmp da error al tener uno de los campos vacios.
while (strcmp(odom.LatestMessage.ChildFrameId,'robot0')~=1)
    odom.LatestMessage
end