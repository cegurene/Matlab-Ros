%Crear figuras distintas para el láser y el visualizador del VFH
% NOTA: para dibujar sobre una figura concreta, antes de llamar a la
% correspondiente función de dibujo debe convertirse en la figura activa
% utilizando figure(fig_laser) o figure(fig_vfh) respectivamente.
close all
fig_laser = figure; title('LASER')
fig_vfh = figure; title('VFH')

%Crear el objeto VFH
VFH=controllerVFH;

%y ajustamos sus propiedades
%VFH.NumAngularSectors=;
VFH.DistanceLimits = [0.05 2.5];
VFH.RobotRadius = 0.17;
VFH.SafetyDistance = 0.5;
VFH.MinTurningRadius = 0.15;
%VFH.TargetDirectionWeight = ;
%VFH.CurrentDirectionWeight = ;
%VFH.PreviousDirectionWeight = ;
%VFH.HistogramThresholds = ;
VFH.UseLidarScan = true; %para permitir utilizar la notación del scan

%Rellenamos los campos por defecto de la velocidad del robot, para que la lineal
%sea siempre 0.1 m/s
msg_vel.linear.x = 0.05;
msg_vel.lineary.y = 0;
msg_vel.linear.z = 0;
msg_vel.angular.x = 0;
msg_vel.angular.y = 0;
msg_vel.angular.z = 0;

firstLoopClosure = false;

%Bucle de control infinito
while(1)
    %Leer y dibujar los datos del láser en la figura ‘fig_laser’
    msg_laser = sub_laser.LatestMessage;

    scans = rosReadLidarScan(msg_laser);
    scans = removeInvalidData(scans, 'RangeLimits', [0 11.8]);
    scans = transformScan(scans, [0 0 giro_laser]);

    %Llamar al objeto VFH para obtener la dirección a seguir por el robot para
    %evitar los obstáculos. Mostrar los resultados del algoritmo (histogramas)
    %en la figura ‘fig_vfh’
    targetDir = 0;
    steeringDir = VFH(scans,targetDir);

    %Rellenar el campo de la velocidad angular del mensaje de velocidad con un
    %valor proporcional a la dirección anterior (K=0.1)
    k = 0.15;
    V_ang = k * steeringDir;
    msg_vel.angular.z = V_ang;

    %Publicar el mensaje de velocidad
    send(pub_vel,msg_vel);
    
    %Esperar al siguiente periodo de muestreo
    waitfor(r);

end