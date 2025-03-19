%% GENERACIÓN DE MENSAJE
msg=rosmessage(pub) %% Creamos un mensaje del tipo declarado en "pub" (geometry_msgs/Twist)
% Rellenamos los campos del mensaje para que el robot avance a 0.2 m/s
% Velocidades lineales en x,y y z (velocidades en y o z no se usan en robots
% diferenciales y entornos 2D)
msg.Linear.X=0.5;
msg.Linear.Y=0;
msg.Linear.Z=0;
% Velocidades angulares (en robots diferenciales y entornos 2D solo se utilizará
% el valor Z)
msg.Angular.X=0;
msg.Angular.Y=0;
msg.Angular.Z=0;

%% Inicializamos la primera posición (coordenadas x,y,z)
initpos=odom.LatestMessage.Pose.Pose.Position;
pos=odom.LatestMessage.Pose.Pose.Position;
aux2 = 0;
%% Bucle de control infinito
while (1)
    aux1=abs(odom.LatestMessage.Pose.Pose.Position.X - pos.X);
    pos=odom.LatestMessage.Pose.Pose.Position;
    if(aux1 > aux2)
        aux1
        aux2 = aux1;
    end
    send(pub, msg);
    % Temporización del bucle según el parámetro establecido en r
    waitfor(r);
end

