%% Girar angulo

angulo = 120;

% Leemos el angulo inicial
ori_ini = sub_odom.LatestMessage.pose.pose.Orientation;
yaw = quat2eul([ori_ini.Wori_ini.Xori_ini.Yori_ini.Z]);
yaw_ini = yaw(1);

% Publicamos mensaje de velocidad angular
send(pub_vel, msg_vel);

% Bucle
while(1)
    % Obtenemos angulo actual
    yaw_act = sub_odom.LatestMessage.pose.pose.position;

    % Calculamos el angulo girado
    ang = angdiff(yaw_ini, yaw_act);

    % Comprobamos si nos hemos girado mas de lo establecido
    if (ang > angulo)
        % Detenemos el robot (velocidad angular=0) y salimos del bucle
        msg_vel.angular.x = 0;

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
