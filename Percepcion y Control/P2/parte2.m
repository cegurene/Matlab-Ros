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
K_ori = 0.8;      % NUEVO: Ganancia para error de orientación
K_dist = 0.6;     % NUEVO: Ganancia para error lateral

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
    
    % Obtener posición y sonar
    pos = odom.LatestMessage.Pose.Pose.Position;
    msg_sonar0 = receive(sonar0);
    dist = double(msg_sonar0.Range_) - 0.105;
    
    % Limitar la medida
    if dist > 5
        dist = 5;
    end
    
    % Distancia avanzada desde la última iteración
    dx = pos.X - lastpos.X;
    dy = pos.Y - lastpos.Y;
    distav = sqrt(dx^2 + dy^2);
    
    % Calcular errores SOLO si hay avance
    if distav > 0.001 && i > 1
        Eori = atan((dist - lastdist) / distav);
    else
        Eori = 0;
    end
    
    Edist = dist - D;
    
    % Control angular
    consigna_vel_ang = K_ori * Eori + K_dist * Edist;
    
    % Saturación
    max_w = 0.5;
    %consigna_vel_ang = max(-max_w, min(consigna_vel_ang, max_w));
    
    % Velocidad lineal constante
    consigna_vel_linear = 0.3;
    
    % Condición de parada (opcional o dejar fuera si el robot debe seguir indefinidamente)
    if i > 20
        if (abs(Edist) < 0.01) && (abs(Eori) < 0.01)
            break
        end
    end

    fprintf('i = %d | dist = %.3f | lastdist = %.3f | distav = %.4f\n', i, dist, lastdist, distav);
    fprintf('Eori = %.4f | Edist = %.4f | w = %.4f\n', Eori, Edist, consigna_vel_ang);
    fprintf('\n');
    
    % Aplicar velocidades al robot
    msg_vel.Linear.X = consigna_vel_linear;
    msg_vel.Angular.Z = consigna_vel_ang;
    send(pub, msg_vel);
    
    % Guardar variables para la siguiente iteración
    lastpos = pos;
    lastdist = dist;
    lastdistav = distav;
    
    % Guardar datos
    medidas(1,i) = dist;
    medidas(2,i) = lastdist;
    medidas(3,i) = distav;
    medidas(4,i) = Eori;
    medidas(5,i) = Edist;

    i = i+1;
    waitfor(r);
end

save('medidas.mat', 'medidas');

msg_vel.Linear.X = 0;
msg_vel.Linear.Y = 0;
msg_vel.Linear.Z = 0;
msg_vel.Angular.X = 0;
msg_vel.Angular.Y = 0;
msg_vel.Angular.Z = 0;

% Comando de velocidad
send(pub,msg_vel);

%% DESCONEXIÓN DE ROS
rosshutdown;