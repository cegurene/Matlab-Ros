%% INICIALIZACIÓN DE ROS
% Se definen las variables de entorno ROS_MASTER_URI (ip del Master) y ROS_IP (IP
% de la máquina donde se ejecuta Matlab). Si se está conectado a la misma red, la
% variable ROS_IP no es necesario definirla.
clear all;
close all;
rosshutdown;
setenv('ROS_MASTER_URI','http://172.29.30.179:11311') %IP del robot
setenv('ROS_IP','172.29.29.65') %Mi IP
rosinit % Inicialización de ROS

%% DECLARACIÓN DE SUBSCRIBERS
odom=rossubscriber('/pose'); % Subscripción a la odometría
laser=rossubscriber('/scan'); % Subscripción al laser
sonar0=rossubscriber('/sonar_0'); % Subscripción al sonar 0
%sonar1...
pause(2)

%% DECLARACIÓN DE PUBLISHERS
pub = rospublisher('/cmd_vel', 'geometry_msgs/Twist');
pub_enable = rospublisher('/cmd_motor_state', 'std_msgs/Int32')
msg_enable_motor = rosmessage(pub_enable);

msg_enable_motor.Data=1;
send(pub_enable, msg_enable_motor);

%% Definimos la perodicidad del bucle (10 hz)
r = robotics.Rate(10);

%% Nos aseguramos recibir un mensaje relacionado con el robot "robot0"
pause(1); % Esperamos 1 segundo para asegurarnos que ha llegado algún mensaje odom,
% porque sino ls función strcmp da error al tener uno de los campos vacios.
while (strcmp(odom.LatestMessage.ChildFrameId,'base_link')~=1)
    odom.LatestMessage
end