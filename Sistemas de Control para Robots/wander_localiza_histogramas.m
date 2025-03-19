%% Cargar el mapa
load('mapa_simulador_slam.mat');
show(map);

startPose = [0,-4.5];
goalPose = [12.5,4];
hold on
plot(startPose, goalPose);

umbralx =0.005;
umbraly=0.005;
umbralyaw=0.005;

%% Crear el objeto VFH
VFH=controllerVFH;

fig_laser = figure;  
title('LASER');
fig_vfh = figure;  
title('VFH');
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

%% Inicializar el localizador AMCL
amcl = monteCarloLocalization;

odometryModel = odometryMotionModel;
odometryModel.Noise = [0.2 0.2 0.2 0.2];
rangeFinderModel = likelihoodFieldSensorModel;
rangeFinderModel.SensorLimits = [0 11.8];
rangeFinderModel.Map = map;

amcl.UseLidarScan = true;
amcl.MotionModel = odometryModel;
amcl.SensorModel = rangeFinderModel;

rangeFinderModel.SensorPose = [0.005 0 0];
amcl.UpdateThresholds = [0.2,0.2,0.2];
amcl.ResamplingInterval = 1;

amcl.ParticleLimits = [10000, 30000];           % Minimum and maximum number of particles
amcl.GlobalLocalization = false;      % global = true      local=false
amcl.InitialPose = [0,-4.5,0];             % Initial pose of vehicle  
%amcl.InitialCovariance = diag([10 10 10])*0,5; % Covariance of initial pose 0,2-0,5
amcl.InitialCovariance = eye(3)*0.5
visualizationHelper = ExampleHelperAMCLVisualization(map);

%% Rellenamos los campos por defecto de la velocidad del robot, para que la lineal
%sea siempre 0.1 m/s
msg_vel.linear.x = 0.1;
msg_vel.lineary.y = 0;
msg_vel.linear.z = 0;
msg_vel.angular.x = 0;
msg_vel.angular.y = 0;
msg_vel.angular.z = 0;

maxLidarRange = 11.8 ;
mapResolution = 20 ;
slamObj = lidarSLAM(mapResolution, maxLidarRange)
slamObj.LoopClosureThreshold = 200 ;
slamObj.LoopClosureSearchRadius = 3;

firstLoopClosure = false;

%% Bucle de control infinito
i=1;
while(1)
    %Leer y dibujar los datos del láser en la figura ‘fig_laser’
    msg_laser = sub_laser.LatestMessage;

    scans = rosReadLidarScan(msg_laser);
    scans = removeInvalidData(scans, 'RangeLimits', [0 11.8]);
    scans = transformScan(scans, [0 0 giro_laser]);

    figure(fig_laser); % Activar la figura para el láser
    plot(scans); % Dibujar los datos del láser

    %% Leer la odometría
    msg_odom = sub_odom.LatestMessage;

    %% Obtener la posición pose=[x,y,yaw] a partir de la odometría anterior
    odomQuat = [msg_odom.pose.pose.orientation.w, msg_odom.pose.pose.orientation.x, ...
    msg_odom.pose.pose.orientation.y, msg_odom.pose.pose.orientation.z]; %Leer orientación del mensaje de odometría 
    odomRotation = quat2eul(odomQuat); %Convertir cuaternio a ángulos de euler
    % Establecer la pose actual [x, y, theta] del robot
    pose_robot = [msg_odom.pose.pose.position.x msg_odom.pose.pose.position.y odomRotation(1)];

    %% Ejecutar amcl para obtener la posición estimada estimatedPose y la
    %covarianza estimatedCovariance (mostrar la última por pantalla para
    %facilitar la búsqueda de un umbral)
    
    % Update estimated robot's pose and covariance using new odometry and
    % sensor readings.
    [isUpdated,estimatedPose, estimatedCovariance] = amcl(pose_robot, scans);
    estimatedCovariance
    

    %% Si la covarianza está por debajo de un umbral, el robot está localizado y
    %finaliza el programa
    if (estimatedCovariance(1,1)<umbralx && estimatedCovariance(2,2)<umbraly && estimatedCovariance(3,3)<umbralyaw)
        disp("Robot Localizado");
        %break;
    end

    %wander(wanderHelper);

    %% Dibujar los resultados del localizador con el visualizationHelper
    if isUpdated
        i = i + 1
        plotStep(visualizationHelper, amcl, estimatedPose, scans, i)
    end

    %% Llamar al objeto VFH para obtener la dirección a seguir por el robot para
    %evitar los obstáculos. Mostrar los resultados del algoritmo (histogramas)
    %en la figura ‘fig_vfh’
    targetDir = 0;
    steeringDir = VFH(scans,targetDir);

    %% Rellenar el campo de la velocidad angular del mensaje de velocidad con un
    %valor proporcional a la dirección anterior (K=0.1)
    k = 0.15;
    V_ang = k * steeringDir;
    msg_vel.angular.z = V_ang;

    figure(fig_vfh); % Activar la figura del VFH
    show(VFH);

    %% Publicar el mensaje de velocidad
    send(pub_vel,msg_vel);
    
    %% Esperar al siguiente periodo de muestreo
    waitfor(r);

end