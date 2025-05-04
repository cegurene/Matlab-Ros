close all;
clear all;
rosshutdown;
setenv('ROS_MASTER_URI','http://192.168.1.138:11311'); %IP del simulador
setenv('ROS_ IP','192.168.1.49'); %Mi IP
rosinit % Inicialización de ROS en la IP correspondiente

%% DECLARACIÓN DE SUBSCRIBERS
odom = rossubscriber('/robot0/odom'); % Subscripción a la odometría
sub_laser = rossubscriber('/robot0/laser_1'); % Subscripción al láser

%% DECLARACIÓN DE PUBLISHERS
pub = rospublisher('/robot0/cmd_vel', 'geometry_msgs/Twist'); % Publicador de velocidades
msg_vel = rosmessage(pub); % Creamos un mensaje del tipo geometry_msgs/Twist

%% Definimos la perodicidad del bucle (10 Hz)
r = robotics.Rate(10);
waitfor(r);

%% Esperamos entre 2 y 5 segundos antes de leer el primer mensaje
pause(3);

%% Nos aseguramos de recibir un mensaje relacionado con el robot
while (strcmp(odom.LatestMessage.ChildFrameId,'robot0') ~= 1)
    odom.LatestMessage
end

giro_laser = 0;

