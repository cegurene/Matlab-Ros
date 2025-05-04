%% Declaracion de SUBSCRIBERS (a la odometria y el laser)
%Odometria
sub_odom= ros2subscriber(node_matlab, '/odom', 'Reliability','besteffort');

%Laser
sub_laser= ros2subscriber(node_matlab, '/laser1');

%Imagen
%sub_camara= ros2subscriber(node_matlab, '/camera1');

%% Declaracion de PUBLISHERS (a la velocidad)
%Velocidad
pub_vel= ros2publisher(node_matlab, '/cmd_vel', 'geometry_msgs/Twist');

%% Creacion del MENSAJE de velocidad
%Velocidad
msg_vel= ros2message(pub_vel);

%% PERIODICIDAD del timer
r = rateControl(10);

giro_laser = 0;