% Borrar variables y cerrar figuras
clear all;
close all;

% Establecer el dominio de ROS
setenv("ROS_DOMAIN_ID","4")

% Crear nodo de ros
node_matlab=ros2node("/node_matlab");
pause(10)  %10 segundos

% Mostramos los nodos activos de ros2
disp("Nodos activos:")
ros2 node list

% Mostramos los topics activos de ros2
disp("Topics:")
ros2 topic list
