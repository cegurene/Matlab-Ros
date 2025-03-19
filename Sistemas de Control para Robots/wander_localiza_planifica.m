%% Definir la posicion de destino
endLocation = [12.5,4];

%% Cargar el mapa
load('mapa_simulador_slam.mat');
%show(map)

umbralx =0.05;
umbraly=0.05;
umbralyaw=0.1;

%% Crear el objeto VFH…y ajustar sus propiedades
VFH=controllerVFH;
VFH.NumAngularSectors=180;
VFH.DistanceLimits = [0.05 2.5];
VFH.RobotRadius = 0.17;
VFH.SafetyDistance = 1;
VFH.MinTurningRadius = 0.15;
VFH.TargetDirectionWeight = 5;
VFH.CurrentDirectionWeight = 2;
VFH.PreviousDirectionWeight = 2;
VFH.HistogramThresholds = [3 10];
VFH.UseLidarScan = true; %para permitir utilizar la notación del scan

%% Inicializar el localizador AMCL (práctica 1)
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
amcl.InitialCovariance = eye(3)*1;
visualizationHelper = ExampleHelperAMCLVisualization(map);

%% Rellenamos los campos por defecto de la velocidad del robot, para que la lineal
%sea siempre 0.1 m/s
msg_vel.linear.x = 0.1;
msg_vel.lineary.y = 0;
msg_vel.linear.z = 0;
msg_vel.angular.x = 0;
msg_vel.angular.y = 0;
msg_vel.angular.z = 0;

%% Bucle de control infinito
i=1;
while(1)
    %% Leer y dibujar los datos del láser en la figura ‘fig_laser’
    msg_laser = sub_laser.LatestMessage;
    scans = rosReadLidarScan(msg_laser);
    scans = removeInvalidData(scans, 'RangeLimits', [0 11.8]);
    scans = transformScan(scans, [0 0 giro_laser]);

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
    [isUpdated,estimatedPose, estimatedCovariance] = amcl(pose_robot, scans);
    estimatedCovariance

    %% Si la covarianza está por debajo de un umbral, el robot está localizado y
    %finaliza el programa
    if (estimatedCovariance(1,1)<umbralx && estimatedCovariance(2,2)<umbraly && estimatedCovariance(3,3)<umbralyaw)
        disp("Robot Localizado");
        break;
    end

    %% Dibujar los resultados del localizador con el visualizationHelper
    if isUpdated
        i = i + 1;
        plotStep(visualizationHelper, amcl, estimatedPose, scans, i)
    end

    %% Llamar al objeto VFH para obtener la dirección a seguir por el robot para
    %evitar los obstáculos. Mostrar los resultados del algoritmo (histogramas)
    %en la figura ‘fig_vfh’
    targetDir = 0;
    steeringDir = VFH(scans,targetDir);

    %% Rellenar el campo de la velocidad angular del mensaje de velocidad con un
    %valor proporcional a la dirección anterior (K=0.1)
    k = 1;
    V_ang = k * steeringDir;
    msg_vel.angular.z = V_ang;

    %% Publicar el mensaje de velocidad
    send(pub_vel,msg_vel);

    %% Esperar al siguiente periodo de muestreo
    waitfor(r);
end

%%%%%%%%%%% AL SALIR DE ESTE BUCLE EL ROBOT YA SE HA LOCALIZADO %%%%%%%%%%
%%%%%%%%%%% COMIENZA LA PLANIFICACIÓN GLOBAL %%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Parar el robot, para que no avance mientras planificamos
msg_vel.linear.x = 0;
msg_vel.angular.z = 0;
send(pub_vel,msg_vel);

%% Hacer una copia del mapa, para “inflarlo” antes de planificar
cpMap= copy(map);
inflate(cpMap,0.25);

%% Crear el objeto PRM y ajustar sus parámetros
planner = mobileRobotPRM;
planner.Map=cpMap;
planner.NumNodes=750;
planner.ConnectionDistance = 4;

%% Obtener la ruta hacia el destino desde la posición actual del robot y mostrarla
%en una figura
startLocation= [estimatedPose(1,1), estimatedPose(1,2)];
ruta=findpath(planner,startLocation,endLocation);
figure;
show(planner);