%% Localize Turtlebot using Monte Carlo Localization
%% Introduction
% This example demonstrates an application of the Monte Carlo Localization (MCL) 
% algorithm.
% 
% Adaptive Monte Carlo Localization (AMCL) is the variant of MCL implemented 
% in <docid:nav_ref.bu31hfz-1 |monteCarloLocalization|>. AMCL dynamically adjusts 
% the number of particles based on KL-distance [1] to ensure that the particle 
% distribution converge to the true distribution of robot state based on all past 
% sensor and motion measurements with high probability.
% 
% The current MATLAB® AMCL implementation can be applied to any differential 
% drive robot equipped with a range finder.
%% Load the map of the simulation world /real environment
% Load a binary occupancy grid of the office environment. The map is generated 
% by driving TurtleBot inside the office environment. The map is constructed using 
% range-bearing readings from laser sensor and odometry poses. 

load mapa_pasillo_slam.mat
%map=map_modified;
show(map);
%% Setup the laser sensor model and Turtlebot motion model
% TurtleBot can be modeled as a differential drive robot and its motion can 
% be estimated using odometry data. The |Noise| property defines the uncertainty 
% in robot's rotational and linear motion. Increasing the |odometryModel.Noise| 
% property will allow more spread when propagating particles using odometry measurements. 
% Please refer to <docid:robotics_ref.bu359h6-1 |robotics.OdometryMotionModel|> 
% for property details.

odometryModel = odometryMotionModel;
odometryModel.Noise = [0.2 0.2 0.2 0.2];
%% 
% The Lidar sensor model should be tuned to match the actual sensor property 
% to achieve better test results. The property |SensorLimits| defines the minimum 
% and maximum range of sensor readings. The property |Map| defines the occupancy 
% map used for computing likelihood field.Please refer to <docid:nav_ref.bu31hrp-1 
% |likelihoodFieldSensorModel|> for property details.

rangeFinderModel = likelihoodFieldSensorModel;
rangeFinderModel.SensorLimits = [0 11.8];
rangeFinderModel.Map = map;
%% 
% Set |rangeFinderModel.SensorPose| to the coordinate transform of the fixed 
% laser sensor with respect to the robot base. 

rangeFinderModel.SensorPose = [0.005 0 0];
%% Initialize AMCL object
% Instantiate an AMCL object |amcl|. See <docid:nav_ref.bu31hfz-1 monteCarloLocalization> 
% for more information on the class.

amcl = monteCarloLocalization;
amcl.UseLidarScan = true;
%% 
% Assign the |MotionModel| and |SensorModel| properties in the |amcl| object.

amcl.MotionModel = odometryModel;
amcl.SensorModel = rangeFinderModel;
%% 
% The particle filter only updates the particles when the robot's movement exceeds 
% the |UpdateThresholds|, which defines minimum displacement in [x, y, yaw] to 
% trigger filter update. This prevents too frequent updates due to sensor noise. 
% Particle resampling happens after the |amcl.ResamplingInterval| filter updates. 
% Using larger numbers leads to slower particle depletion at the price of slower 
% particle convergence as well.

amcl.UpdateThresholds = [0.2,0.2,0.2];
amcl.ResamplingInterval = 1;
%% Configure AMCL object for localization with initial pose estimate.
% |amcl.ParticleLimits| defines the lower and upper bound on the number of particles 
% that will be generated during the resampling process. Note that global localization 
% may need significantly more particles compared to localization with an initial 
% pose estimate. If the robot knows its initial pose with some uncertainty, such 
% additional information can help AMCL localize robots faster with a less number 
% of particles, i.e. you can use a smaller value of upper bound in |amcl.ParticleLimits|.
% 
% Now set |amcl.GlobalLocalization| to false and provide an estimated initial 
% pose to AMCL. By doing so, AMCL holds the initial belief that robot's true pose 
% follows a Gaussian distribution with a mean equal to |amcl.InitialPose| and 
% a covariance matrix equal to |amcl.InitialCovariance|. Initial pose estimate 
% should be obtained according to your setup. This example helper retrieves the 
% robot's current true pose from Gazebo.
% 
% Please refer to section *Configure AMCL object for global localization* for 
% an example on using global localization.

amcl.ParticleLimits = [10000, 50000];           % Minimum and maximum number of particles
amcl.GlobalLocalization = true;      % global = true      local=false
amcl.InitialPose = [2,0,0];             % Initial pose of vehicle  
amcl.InitialCovariance = diag([1 1 1])*0,5; % Covariance of initial pose 0,2-0,5
%% Setup helper for visualization and driving AmigoBot.
% Setup ExampleHelperAMCLVisualization to plot the map and update robot's estimated 
% pose, particles, and laser scan readings on the map.

visualizationHelper = ExampleHelperAMCLVisualization(map);
%% Localization procedure
% The AMCL algorithm is updated with odometry and sensor readings at each time 
% step when the robot is moving around. Please allow a few seconds before particles 
% are initialized and plotted in the figure. In this example we will run |numUpdates| 
% AMCL updates. If the robot doesn't converge to the correct robot pose, consider 
% using a larger |numUpdates|.

i=1;
while (1)
    % Receive laser scan and odometry message.
    msg_laser = sub_laser.LatestMessage;
    msg_odom = sub_odom.LatestMessage;
    
    %Crear objeto para almacenar el escaneo LiDAR 2-D
    scans=rosReadLidarScan(msg_laser); % Extraer datos del mensaje del laser
    scans = removeInvalidData(scans,'RangeLimits',[0 11.5]); % Quitar datos fuera de rango
    scans = transformScan(scans,[0 0 giro_laser]);  % Girar datos laser si es necesario
    
    
    % Compute robot's pose [x,y,yaw] from odometry message.
    
    odomQuat = [msg_odom.pose.pose.orientation.w, msg_odom.pose.pose.orientation.x, ...
    msg_odom.pose.pose.orientation.y, msg_odom.pose.pose.orientation.z]; %Leer orientación del mensaje de odometría 
    odomRotation = quat2eul(odomQuat); %Convertir cuaternio a ángulos de euler
    % Establecer la pose actual [x, y, theta] del robot
    pose_robot = [msg_odom.pose.pose.position.x msg_odom.pose.pose.position.y odomRotation(1)];

    % Update estimated robot's pose and covariance using new odometry and
    % sensor readings.
    [isUpdated,estimatedPose, estimatedCovariance] = amcl(pose_robot, scans);
    
    % Drive robot to next pose.
    %wander(wanderHelper);
    
    % Plot the robot's estimated pose, particles and laser scans on the map.
    if isUpdated
        i = i + 1
        plotStep(visualizationHelper, amcl, estimatedPose, scans, i)
    end

    % Wait for control rate to ensure 10 Hz rate
    waitfor(r);
end
%%