%% Avanzar distancia
distancia = 1;

% Leemos la posicion inicial
initpos = sub_odom.LatestMessage.pose.pose.position;

% Publicamos mensaje de velocidad
msg_vel.linear.x = 0.2;
msg_vel.linear.y = 0;
msg_vel.linear.z = 0;

msg_vel.angular.x = 0;
msg_vel.angular.y = 0;
msg_vel.angular.z = 0;

send(pub_vel, msg_vel);

% Bucle
while(1)
    % Obtenemos posicion actual
    pos = sub_odom.LatestMessage.pose.pose.position;
    
    % Calculamos la distancia desplazada
    distRecorrrida = sqrt((initpos.x - pos.x)^2 + (initpos.y - pos.y)^2);
    
    % Comprobamos si nos hemos desplazado mas de lo establecido
    if (distRecorrrida > distancia)
        % Detenemos el robot (velocidad lineal=0) y salimos del bucle
        msg_vel.linear.x = 0;
        
        % Comando de velocidad
        send(pub_vel, msg_vel);
        
        % Salimos del bucle
        break;
    else
        % Comando de velocidad
        send(pub_vel, msg_vel);
    end

    % Temporizacion del bucle segun el parametro establecido
    % (en 'ini_simulador')
    waitfor(r)

end
