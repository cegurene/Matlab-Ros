%% INICIALIZACIÓN DE ROS
close all;
clear all;
rosshutdown; % Cierra cualquier conexión previa de ROS
setenv('ROS_MASTER_URI','http://192.168.1.138:11311'); % IP del simulador STDR
setenv('ROS_ IP','192.168.1.49'); % IP local de tu máquina
rosinit % Inicia la conexión con el nodo maestro de ROS

%% SUBSCRIPCIÓN A TOPICS DE ROS
odom = rossubscriber('/robot0/odom');
laser = rossubscriber('/robot0/laser_1');

%% PUBLICADOR DE VELOCIDAD
pub = rospublisher('/robot0/cmd_vel', 'geometry_msgs/Twist');
msg_vel = rosmessage(pub);

%% FRECUENCIA DEL BUCLE
r = robotics.Rate(10);
waitfor(r);

%% ESPERA PARA RECIBIR MENSAJES INICIALES
pause(3);
while (strcmp(odom.LatestMessage.ChildFrameId,'robot0') ~= 1)
    odom.LatestMessage
end

%% PARÁMETROS DE CONTROL
vel_lineal_max = 1.0;
vel_angular_max = 1.0;

slamObj = lidarSLAM(17,7.8,200);     % Object that performs SLAM (map resolution 20 cells per meter, 8 m max lidar range, max of 1000 scans)
slamObj.LoopClosureThreshold = 350; % Raise threshold to prevent smearing

robotLeftFlag = false; % Track if the robot has left the start area
while true
    [lidarMsg,status,~] = receive(laser); % Get the current lidar scan from the robot

    if status % If the scan is good, do SLAM
        angles = double(lidarMsg.AngleMin:lidarMsg.AngleIncrement:lidarMsg.AngleMax);
        ranges = double(lidarMsg.Ranges);
        scan = lidarScan(ranges,angles);                                                      % Build the scan object
        removeInvalidData(scan,RangeLimits=[1/slamObj.MapResolution slamObj.MaxLidarRange]);  % Remove invalid data to avoid errors
        addScan(slamObj,scan);                                                                % Add the scan
        show(slamObj);

        [~,poses] = scansAndPoses(slamObj);              % Get the robot poses from the SLAM map
        currentDistance = sqrt(sum(poses(end,1:2).^2));  % Take the most recent pose (current position of the robot) and calculate how far it is from the start

        if ~robotLeftFlag && currentDistance >= 2  % If the robot has left the start area, begin checking if the robot has returned to the start area
            robotLeftFlag = true;
        end
        if robotLeftFlag && currentDistance <= 0.5 % If the robot has completed a lap, end the control loop
            send(stopRobotPublisher,rosmessage(stopRobotPublisher))
            break % End control loop
        end
    end

    send(startRobotPublisher,rosmessage(startRobotPublisher)) % Tell the navigation node to move the robot
    waitfor(r);
end