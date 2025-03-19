%% Funcion avanzar
function avanzar(distancia)
    odom=rossubscriber('/robot0/odom'); % Subscripción a la odometría
    pub = rospublisher('/robot0/cmd_vel', 'geometry_msgs/Twist');
    r = robotics.Rate(10);
    pause(2)

    % Parámetros de movimiento
    msg = rosmessage(pub);
    msg.Linear.X=0.1;
    msg.Linear.Y=0;
    msg.Linear.Z=0;
    % Velocidades angulares (solo se utiliza el valor Z)
    msg.Angular.X=0;
    msg.Angular.Y=0;
    msg.Angular.Z=0;
    
    initpos=odom.LatestMessage.Pose.Pose.Position;
    actualpos=odom.LatestMessage.Pose.Pose.Position;

    % Avanzar hasta alcanzar la distancia deseada
    while (1)
        actualpos=odom.LatestMessage.Pose.Pose.Position;
        distancia_recorrida = sqrt((initpos.X - actualpos.X)^2 + (initpos.Y - actualpos.Y)^2);

        if abs(distancia_recorrida - distancia) <= 0.05
            msg.Linear.X=0;
            send(pub, msg);
            break;
        end
        
        send(pub, msg);
        % Temporización del bucle según el parámetro establecido en r
        waitfor(r);
    end

end

%% Funcion girar
function girar(angulo)   
    odom=rossubscriber('/robot0/odom'); % Subscripción a la odometría
    pub = rospublisher('/robot0/cmd_vel', 'geometry_msgs/Twist');
    r = robotics.Rate(10);
    pause(2)
    % Recibir mensaje de odometría inicial
    init_orient = odom.LatestMessage.Pose.Pose.Orientation;

    % Convertir cuaternión inicial a ángulo de Euler (yaw)
    quat0 = [init_orient.W, init_orient.X, init_orient.Y, init_orient.Z];
    eul0 = quat2eul(quat0, 'ZYX');  % Devuelve [roll, pitch, yaw]
    yaw0 = rad2deg(eul0(1)); % Extraer yaw en grados
    
    % Calcular ángulo de destino
    angulo_deseado = angulo + yaw0;
    angulo_deseado = mod(angulo_deseado + 180, 361) - 180;

    % Crear mensaje de velocidad
    msg = rosmessage(pub);
    msg.Linear.X = 0;
    msg.Linear.Y = 0;
    msg.Linear.Z = 0;
    msg.Angular.X = 0;
    msg.Angular.Y = 0;
    msg.Angular.Z = 0.1; % Velocidad de giro
    if angulo < 0
        msg.Angular.Z = -0.1;
    end

    % Enviar comando de giro
    send(pub, msg);

    % Bucle hasta alcanzar el ángulo deseado
    while 1 
        % Leer odometría actual
        actual_orient = odom.LatestMessage.Pose.Pose.Orientation;
        quat = [actual_orient.W, actual_orient.X, actual_orient.Y, actual_orient.Z];
        eul = quat2eul(quat, 'ZYX');
        yaw_actual = rad2deg(eul(1));
        
        % Calcular la diferencia de ángulo (yaw)
        diferencia_giro = abs(yaw_actual - yaw0);

        if abs(yaw_actual - angulo_deseado) <= 0.5
            msg.Angular.Z=0;
            send(pub, msg);
            break;
        end
        
        waitfor(r); % Pequeña pausa para evitar sobrecarga
    end
end

%% Funcion simulador
function simulador()

    % Empezamos en la posicion (9,1)
    girar(180);
    avanzar(2);
    
    % Estamos en la posicion (7,1)
    girar(-90);
    avanzar(2);
    
    % Estamos en la posicion (7,3)
    girar(90);
    avanzar(4);
    
    % Estamos en la posicion (3,3)
    girar(90);
    avanzar(2.25);
end

%% Funcion robot real
function real()

    posicion_inicial = odom.LatestMessage.Pose.Pose.Position;

    % Recorrido
    avanzar(2);
    girar(90);
    avanzar(1);
    girar(-90);
    avanzar(1);
    
    % Comprobamos el error
    actualpos=odom.LatestMessage.Pose.Pose.Position;
    
    error = sqrt((posicion_inicial.X - actualpos.X)^2 + (posicion_inicial.Y - actualpos.Y)^2);
    error
end

simulador();

