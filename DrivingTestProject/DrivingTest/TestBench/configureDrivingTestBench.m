classdef configureDrivingTestBench
    %configureDrivingTestBench Initializes the DrivingTestBench.slx model
    %by creating data in base workspace.
    %
    %   configureDrivingTestBench properties:
    %
    % TestStartTime            - Start time when AEB Controller
    %                            is enabled for the test. Units
    %                            are in seconds.
    %
    % ObjectDetections         - Object detections variant name.
    %
    %  ScenarioSimulationObj  - Scenario simulation object as returned by
    %                            Simulink.ScenarioSimulation.find function.
    %
    %  StepSize               - Sample time of the DrivingTestBench model.
    %
    %  EgoActorID              - Actor ID of the ego vehicle.
    %
    %  RefPathSize             - Number of points on the ego reference
    %                            path.
    %
    %  NumTargetActors         - Number of target actors in the RoadRunner
    %                            scenario.
    %
    %  PredictionHorizon       - Controller prediction horizon.
    %
    %  Vehicle                 - Test vehicle configuration parameters structure with the following elements
    %
    %                             Fidelity                     - Fidelity of the vehicle 3DOF or 14DOF
    %                             Mass                         - Vehicle mass
    %                             StartFromRest                - Flag to determine whether to start the vehicle from rest or with constant test speed.
    %                             YawMomentOfInertia           - Yaw moment inertia of vehicle
    %                             FrontTireCorneringStiffness  - Front tire cornering stiffness of vehicle.
    %                             RearTireCorneringStiffness   - Rear tire cornering stiffness of vehicle.
    %
    %  EnableVisualization     - Flag to enable or disable runtime visualization.
    %
    % configureDrivingTestBench methods:
    %
    % setup          - Create a bus and initialize variables and parameters
    %                  in the base workspace to simulate the test bench
    %                  model.
    % cleanup        - Clear the bus, variables, and parameters of the test
    %                  bench model.
    %
    % Optional inputs for the setup method
    %   ObjectDetections:
    %     - Name of the object detection variant specified as
    %       "vision-radar" and "groundtruth" (default).
    %   TestStartTime:
    %     - Test start time  in seconds at which the AEB controller
    %       is enabled. The acceptable range is 0 to 100 seconds.
    %       This is only valid when AEB controller is on. 
    %   Vehicle:
    %     - Vehicle params structure with the following fields.
    %                             Fidelity                    - Fidelity of the vehicle specified as 3DOF or 14DOF
    %                             Mass                        - Vehicle mass
    %                             StartFromRest               - Flag to determine whether to start the vehicle from rest or with constant test speed.
    %                             YawMomentOfInertia          - Yaw moment inertia of vehicle
    %                             FrontTireCorneringStiffness - Front tire cornering stiffness of vehicle.
    %                             RearTireCorneringStiffness  - Rear tire cornering stiffness of vehicle.
    %  Controller:
    %     - Name of the Controller specified as 
    %     "AEB" or "LKA" or "Trajectory Follower"(default).
    %      AEB - Autonomous Emergency Braking.
    %      LKA - Lane Keep Assist.
    %      Trajectory Follower - Trajectory following controller.
    %      
    %
    %  StepSize:
    %     - Model step size.
    %
    %  EnableVisualization:
    %     - Run-time visualization enable or disable flag.
    %
    % Example:
    %
    % drivingConfigObj  = configureDrivingTestBench;
    % setup(drivingConfigObj, ObjectDetections="groundtruth", TestStartTime=3.5);
    % cleanup(drivingConfigObj);
    %

    % Copyright 2023-2025 The MathWorks, Inc.

    properties(Access = private)
        TestStartTime;
        ObjectDetections;
        ScenarioSimulationObj;
        StepSize;
        EgoActorID;
        RefPathSize;
        NumTargetActors;
        PredictionHorizon;
        Vehicle;
        EnableVisualization = 0;
    end

    methods(Static)
        function [vehicleParams] = VehicleParameters(veh)
            arguments
                veh.Fidelity{mustBeMember(veh.Fidelity,...
                                          ["3DOF",...
                                           "14DOF"])}= "14DOF";
            end
            if(strcmp(veh.Fidelity, "14DOF"))
                veh.StartFromRest = false;
            else
                veh.StartFromRest = true;
            end
            veh.Mass = 1575;
            veh.YawMomentOfInertia = 2875;
            veh.FrontTireCorneringStiffness = 19000;
            veh.RearTireCorneringStiffness  = 33000;
            vehicleParams = veh;
        end
    end

    methods
        function setup(obj,nvp)
        %setup function creates required buses, variables for
        %simulating the DrivingTestBench.slx

            arguments
                obj configureDrivingTestBench;

                nvp.Vehicle = struct('Mass',1575, 'YawMomentOfInertia',  2875, ...
                                     'FrontTireCorneringStiffness', 19000, 'RearTireCorneringStiffness', 33000, 'Fidelity', "3DOF", 'StartFromRest',true);

                nvp.TestStartTime (1,1) double {...
                    mustBeInRange(nvp.TestStartTime,0,60)} = 0;

                nvp.ObjectDetections {mustBeMember(nvp.ObjectDetections,...
                                                   ["groundtruth",...
                                                    "vision-radar"])} = "groundtruth";

                nvp.Controller {mustBeMember(nvp.Controller,...
                    ["AEB",...
                    "LKA",...
                    "Trajectory Follower"])} = "Trajectory Follower";

                nvp.StepSize (1,1) double {...
                    mustBeInRange(nvp.StepSize,0.01,0.05)} = 0.03;

                nvp.EnableVisualization (1,1) logical = 0;
            end

            obj.Vehicle = nvp.Vehicle;

            obj.EnableVisualization = nvp.EnableVisualization;

            %% Load test bench model
            drivingTestBenchModel = 'DrivingTestBench';

            wasModelLoaded = bdIsLoaded(drivingTestBenchModel);
            if ~wasModelLoaded
                load_system(drivingTestBenchModel)
            end


            %%
            switch nvp.ObjectDetections
              case "groundtruth"
                objectDetectionChoice = "groundTruth";
              case "vision-radar"
                objectDetectionChoice = "allSensors";
              otherwise
                error("Not a valid input parameters for object detection");
            end

            set_param(drivingTestBenchModel + "/Scenario and Environment/Sensors/Object Detections", "LabelModeActiveChoice", objectDetectionChoice);

            % laneDetectionChoice:
            %     - Name of the lane detection variant specified as
            %       "No Lanes" (default) or "Vision".
            % Use laneDetectionChoice = "Vision" to get lanes information
            %     from the vision detection generator sensor.
            if(nvp.Controller == "LKA")
            laneDetectionChoice = "Vision"; % Vision variant outputs the lane bus from vision sensor.
            else
            laneDetectionChoice = "No Lanes"; % No Lanes variant outputs the empty lane bus.
            end
            set_param(drivingTestBenchModel + "/Scenario and Environment/Sensors/Lanes", "LabelModeActiveChoice", laneDetectionChoice);

            obj.TestStartTime = nvp.TestStartTime;
            obj.ObjectDetections = nvp.ObjectDetections;

            switch nvp.Controller
                case "AEB"
                    assignin('base', 'aebEnable', true);
                    assignin('base', 'lkaEnable', false);
                case "LKA"
                    assignin('base', 'lkaEnable', true);
                    assignin('base', 'aebEnable', false);
                case "Trajectory Follower"
                    assignin('base', 'lkaEnable', false);
                    assignin('base', 'aebEnable', false);
            end

            % Threshold value for the Distance To Line Crossing. 
            % Once the vehicle crosses this threshold the LKA controller triggers
            % to bring back the vehicle into its lane. 
            assignin('base', 'dtleThreshold', 0.1);
 
            assignin('base', 'vehicle', obj.Vehicle.Fidelity);
            vehicleDynamicsBlk = drivingTestBenchModel + "/Vehicle Dynamics/Vehicle Dynamics/14DOF/Vehicle Dynamics 14DOF" ;

            % Set the 14DOF reference model to accelerator mode if it is
            % not already set.
            if(obj.Vehicle.Fidelity == "14DOF" && ~strcmp(get_param(vehicleDynamicsBlk,'SimulationMode'), 'Accelerator'))
                %% Load vehicle dynamics reference model
                vehDynModel = 'VehDyn14DOF';

                wasVehDynModelLoaded = bdIsLoaded(vehDynModel);
                if ~wasVehDynModelLoaded
                    load_system(vehDynModel)
                end
                set_param(vehicleDynamicsBlk,'SimulationMode','accelerator');
                save_system(vehDynModel);
                save_system(drivingTestBenchModel);
            end

            % Initialize simulation sample time (s)
            obj.StepSize = nvp.StepSize;
            assignin('base', 'Ts', obj.StepSize); % simulation sample time  (s)
            assignin('base', 'ControllerStepSize', obj.StepSize); % simulation sample time of controller  (s)

            assignin('base', 'testStartTime', obj.TestStartTime);

            %% Default value initializations
            obj.EgoActorID = 1;
            obj.RefPathSize = 5000; % default number of path points.

            egoSetSpeed = 2.778; % default speed

            egoInitialPose = struct;
            egoInitialPose.ActorID = 1;
            egoInitialPose.Position = [0 0 0];
            egoInitialPose.Velocity = [0 0 0];
            egoInitialPose.Roll = 0;
            egoInitialPose.Pitch = 0;
            egoInitialPose.Yaw = 0;
            egoInitialPose.AngularVelocity = [0 0 0];
            egoInitialPose.vehicleFrameVelocity = [0 0 0];

            numActors = 2;

            initActorProfile = struct(...
                'ActorID',1,...
                'ClassID',1,...
                'Length',4.7,...
                'Width',1.8,...
                'Height',1.4,...
                'OriginOffset',[0 0 0],...
                'FrontOverhang',0,...
                'RearOverhang',0,...
                'Wheelbase',0,...
                'Color',[0 0 0]);

            actorProfiles = repmat(initActorProfile, 1, numActors);
            testStartPosition = [0 0 0]; % default test start position

            %% If RR scenario simulation object is available, load scenario
            % data and compute ego speed, actor profiles, and test start
            % position.
            obj.ScenarioSimulationObj = Simulink.ScenarioSimulation.find('ScenarioSimulation');


            % Create and initialize map from ActorID to index
            actorIdToIndexMap = containers.Map('KeyType', 'double', 'ValueType', 'double');

            % Store the mapping of ActorID of 1 to index ego vehicle by defalut.
            actorIdToIndexMap(1) = obj.EgoActorID;

           if ~isempty(obj.ScenarioSimulationObj)

                % Read actor profiles from RoadRunner Scenario
                worldActor = obj.ScenarioSimulationObj.getScenario();

                world = worldActor.actor_spec.world_spec;

                numActors = length(world.actors);
                obj.NumTargetActors  = numActors-1;
                [actorProfiles, actorIdToIndexMap] = getActorProfiles(obj, world.actors, "RearAxleCenter");
                egoIndexInActorList=1;

                % Find ego ActorID and error out if an associated DrivingTestBench.rrbehavior is not found.
                foundBehavior = false;

                for i = 1:length(world.behaviors)
                    assetRef = world.behaviors(i).asset_reference;

                    if ~isempty(assetRef) && contains(upper(assetRef), upper('DrivingTestBench.rrbehavior'))
                        egoBehavior = world.behaviors(i).id;
                        foundBehavior = true;

                        for j = 1:length(world.actors)
                            actorSpec = world.actors(j).actor_spec;
                            id = str2double(actorSpec.id);

                            if isequal(actorSpec.behavior_id, egoBehavior)
                                obj.EgoActorID = id;
                                egoIndexInActorList = j;

                                if isempty(actorSpec.vehicle_spec) && ~isempty(actorSpec.character_spec)
                                    error(message('variantgenerator:drivingApplication:DrivingTestBehaviorForCharacterActorIsNotSupported'));
                                end

                                break; % Exit the actor loop once the ego actor is found
                            end
                        end

                        break; % Exit the behavior loop once the desired behavior is processed
                    end
                end

                % Error out if no behavior associated with DrivingTestBench.rrbehavior is found
                if ~foundBehavior
                    error(message('variantgenerator:drivingApplication:noVehicleAssociatedWithDrivingTestBehavior'));
                end

                hasTimeData = 0; % Flag to identify if scenario has timingdata
                tolerance = 1e-3; % Define a small tolerance to find if the user modified the timing data.

                % Check the first phase for the ego vehicle in the scenario
                if strcmp(world.scenario.root_phase.whichOneof('type'), 'composite_phase')

                    phase = world.scenario.root_phase.composite_phase.children(egoIndexInActorList, 1);

                    % Traverse through composite phases until you reach an action phase
                    while strcmp(phase.whichOneof('type'), 'composite_phase')
                        phase = phase.composite_phase.children(1, 1);
                    end

                    if strcmp(phase.action_phase.whichOneof('type'),'actor_action_phase')
                        % Path action cannot be defined without speed action
                        if (numel(phase.action_phase.actions)>1) && strcmp(phase.action_phase.actions(2, 1).actor_action.whichOneof('type'),'path_action')

                            path = phase.action_phase.actions(2, 1).actor_action.path_action.path;
                            if ~numel(path)
                                error(message('variantgenerator:drivingApplication:firstPhaseNotSetToPathFollowingMode'));
                            end
                            egoSetSpeed  = phase.action_phase.actions(1, 1).actor_action.speed_action.speed_target.value;
                            protoPathAction = phase.action_phase.actions(2, 1).actor_action.path_action;
                            hasTimeData = size(protoPathAction.timings, 1) && ~(abs(protoPathAction.timings(end, 1).time) <= tolerance);
                            % PathPointTiming data must be provided on the
                            % actor's current path. Enum value 5 indicates
                            % "SPEED_COMPARISON_FROM_PATH".
                            isWaypointTimeDataMode = phase.action_phase.actions(1, 1).actor_action.speed_action.speed_target.speed_reference.speed_comparison == 5;

                        else
                            error(message('variantgenerator:drivingApplication:firstPhaseNotSetToPathFollowingMode'));
                        end
                    else
                        error(message('variantgenerator:drivingApplication:firstPhaseNotSetToPathFollowingMode'));
                    end
                else
                    error(message('variantgenerator:drivingApplication:firstPhaseNotSetToPathFollowingMode'));
                end

                % Set initial speed as set speed and override later if time data exists.
                egoInitialSpeed = egoSetSpeed;

                if(hasTimeData && isWaypointTimeDataMode)
                    % Convert proto path action to path action structure
                    pathAction.PathTarget.Path = [[protoPathAction.path.points.x]' [protoPathAction.path.points.y]' [protoPathAction.path.points.z]'];
                    pathAction.PathTarget.NumPoints = length(pathAction.PathTarget.Path);
                    pathAction.PathTarget.HasTimings = 1;
                    for i = 1:pathAction.PathTarget.NumPoints
                        timings(i).Time = protoPathAction.timings(i).time;
                        timings(i).Speed = protoPathAction.timings(i).speed;
                        timings(i).WaitTime = protoPathAction.timings(i).wait_time;
                    end
                    pathAction.PathTarget.Timings = timings;

                    pathActionAdapter = drivingApplication.internal.path.PathActionAdapter;
                    [~,refPathTimingData] = pathActionAdapter(pathAction);

                    % Position estimation using time based trajectory
                    waypointAdapter = drivingApplication.internal.path.WaypointAdapter;
                    poseEstimator = drivingApplication.internal.path.ActorPoseEstimator;
                    x = 0;
                    y = 0;
                    z = 0;
                    currentTime = 0;

                    while currentTime <= obj.TestStartTime
                        currentTime = currentTime + obj.StepSize;
                        speed = stepImpl(waypointAdapter, obj.StepSize, currentTime, refPathTimingData);
                        [x, y, z, ~, ~, ~] = poseEstimator(pathAction.PathTarget.Path, pathAction.PathTarget.NumPoints, obj.StepSize, speed);
                    end

                    testStartPosition = [x, y, z];


                    if(numel(protoPathAction.timings) - 2 > 0)
                        % Get the vehicle test speed from the path action data.
                        egoSetSpeed = protoPathAction.timings(end-2,1).speed;
                    else
                        egoSetSpeed =  protoPathAction.timings(end,1).speed;
                    end

                    egoInitialSpeed = protoPathAction.timings(1,1).speed;
                    % Ego initial speed. This initial speed of ego vehicle
                    % is used when the vehicle's StartFromRest flag is true.
                    if(egoInitialSpeed < 0.8 && obj.Vehicle.Fidelity == "14DOF")
                        egoInitialSpeed = 0.7;
                    end
                else
                    egoStartXPos = world.actors(egoIndexInActorList, 1).actor_runtime.pose.matrix.col3.x;
                    egoStartYPos = world.actors(egoIndexInActorList, 1).actor_runtime.pose.matrix.col3.y;
                    egoStartZPos = world.actors(egoIndexInActorList, 1).actor_runtime.pose.matrix.col3.z;
                    testStartPosition = [egoStartXPos egoStartYPos egoStartZPos];

                    if(obj.TestStartTime ~= 0)
                        warning(message('variantgenerator:drivingApplication:testStartTimeIgnored'));
                    end
                    
                    % Ego initial speed. This initial speed of ego vehicle
                    % is used when the vehicle's StartFromRest flag is true.
                    if(egoInitialSpeed < 0.8 && obj.Vehicle.Fidelity == "14DOF")
                        egoInitialSpeed = 0.7;
                        warning(message('variantgenerator:drivingApplication:overrideLowInitialSpeedForEgoVehicle', obj.EgoActorID));
                    end
                end

                if(obj.Vehicle.StartFromRest)
                    egoInitialPose = getActorPose(obj, world.actors, egoInitialSpeed);
                else
                    egoInitialPose = getActorPose(obj, world.actors, egoSetSpeed);
                end
            end

            % Set ego velocity (m/s), actor profiles, and test start
            % position
            assignin('base','steadyStateSpeed', egoSetSpeed);
            assignin('base','actorProfiles', actorProfiles);
            assignin('base', "testStartPosition", testStartPosition);
            assignin('base', 'egoActorID', obj.EgoActorID);

            % Assign in max number of actors
            assignin('base', 'maxNumActors', numActors);
            assignin('base', 'numTargetActors',  obj.NumTargetActors);

            % Arc length between interpolated ego path points
            discretizationDistance = 0.2;
            assignin('base','discretizationDistance', discretizationDistance);
            %% Sensor configuration
            % Long Range Radar
            radarParams.Position = [2.2, 0, 0.1];
            radarParams.Rotation =[0 0 0];
            radarParams.DetectionRanges = [1,150];
            radarParams.FieldOfView     = [320,4];
            radarParams.AzRes = 4;
            radarParams.RangeRes = 2.5;
            radarParams.RangeRateRes = 0.5;
            assignin('base','radarParams', radarParams);

            % 1.2MP, FoV = 49 deg
            cameraParams.ImageSize = [480, 640];
            cameraParams.PrincipalPoint = [320, 240];
            cameraParams.FocalLength = [800, 800];
            cameraParams.Position = [2.1,0,1.1];
            cameraParams.Rotation = [0 10 0];
            cameraParams.FieldOfView = [45,45];
            cameraParams.DetectionRanges = [6,150];
            assignin('base','cameraParams', cameraParams);

            %% Tracking and Sensor Fusion Parameters
            if nvp.ObjectDetections == "vision-radar"
                trackingParams.clusterSize = 4;        % Distance for clustering               (m)
                trackingParams.assigThresh = 100;      % Tracker assignment threshold          (N/A)
                trackingParams.M           = 2;        % Tracker M value for M-out-of-N logic  (N/A)
                trackingParams.N           = 3;        % Tracker N value for M-out-of-N logic  (N/A)
                trackingParams.numCoasts   = 3;        % Number of track coasting steps        (N/A)
                trackingParams.numTracks   = 20;       % Maximum number of tracks              (N/A)
                trackingParams.numSensors  = 2;        % Maximum number of sensors             (N/A)

                % Assign TrackingParams struct in base workspace
                assignin('base','trackingParams',trackingParams);
            end

            %% Controller parameters
            maxSteer = 1.13; % Maximum steering angle (rad)
            assignin('base', 'maxSteer', maxSteer);

            minSteer = -1.13; % Minimum steering angle (rad)
            assignin('base', 'minSteer', minSteer);

            minAcc = -3;
            assignin('base','minAcc', minAcc);      % Minimum acceleration   (m/s^2)

            maxAcc = 6;
            assignin('base', 'maxAcc', maxAcc);     % Maximum acceleration   (m/s^2)

            obj.PredictionHorizon = 10; % Number of steps for preview    (N/A)
            assignin('base', 'predictionHorizon', obj.PredictionHorizon);

            controlHorizon = 2;  % The number of MV moves to be optimized at control interval. (N/A)
            assignin('base', 'controlHorizon', controlHorizon);

            assignin('base','maxDecel', -9.83); % Maximum deceleration   (m/s^2)

            assignin('base','defaultSpacing',40);
            assignin('base','timeGap',1.5);            % time gap               (s)

            assignin('base','LaneWidth', single(3.85));

            % Classical ACC parameters inherited from ACC_Model.slx.
            assignin('base','accStandstillGap', 10);      % m
            assignin('base','accTimeHeadway', 1.5);       % s
            assignin('base','accGapGain', 0.6);           % 1/s
            assignin('base','accKp', 1.0);
            assignin('base','accKi', 0.5);
            assignin('base','accMinAcceleration', -5);    % m/s^2
            assignin('base','accMaxAcceleration', 2);     % m/s^2
            assignin('base','accLaneWidth', 3.85);        % m

            
            % Create AEB buses
            createBusObjects(obj);

            % Create the Bus Tracks if the object detection mode is radar.
            if nvp.ObjectDetections == "radar"
                evalin('base','clear(''BusMultiObjectTracker1Tracks'')');
                evalin('base','clear(''BusMultiObjectTracker1'')');

                blk = [drivingTestBenchModel, '/Scenario and Environment/Sensors/Object Detections/Radar/Driving Radar Data Generator'];
                drivingRadarDataGenerator.createBus(blk);
            end
            %% FCW parameters
            FCW.timeToReact  = 1.2;         % driver reaction time           (sec)
            FCW.driverDecel = 4.0;         % driver braking deceleration     (m/s^2)

            % Assign FCW struct in base workspace
            assignin('base','FCW',FCW);

            % AEB parameters
            AEB.PB1Decel = 3.8;            % 1st stage Partial Braking deceleration (m/s^2)
            AEB.PB2Decel = 5.3;            % 2nd stage Partial Braking deceleration (m/s^2)
            AEB.FBDecel  = 9.8;            % Full Braking deceleration              (m/s^2)
            
            if isKey(actorIdToIndexMap, obj.EgoActorID)
                egoIndex = actorIdToIndexMap(obj.EgoActorID);
            else
                error(message('variantgenerator:drivingApplication:egoActorNotFoundInActorProfileList', obj.EgoActorID));
            end

            AEB.headwayOffset = actorProfiles(egoIndex).Length - actorProfiles(egoIndex).RearOverhang; % headway offset                  (m)
            AEB.timeMargin = 0.1;             % headway time margin                    (sec)

            % Assign AEB struct in base workspace
            assignin('base','AEB',AEB);
            
            % Vehicle Parameters
            egoVehDyn = egoVehicleDynamicsParams(obj, egoInitialPose, actorProfiles(egoIndex));
           
            % Dynamics modeling parameters
            egoVehDyn.Mass       = obj.Vehicle.Mass;                                   % Total mass of vehicle                          (kg)
            egoVehDyn.YawMomentOfInertia      = obj.Vehicle.YawMomentOfInertia;        % Yaw moment of inertia of vehicle               (m*N*s^2)
            egoVehDyn.CGToFrontAxle      = egoVehDyn.CGToFrontAxle;                    % Longitudinal distance from c.g. to front axle  (m)
            egoVehDyn.CGToRearAxle      = egoVehDyn.CGToRearAxle;                      % Longitudinal distance from c.g. to rear axle   (m)
            egoVehDyn.Cf      = obj.Vehicle.FrontTireCorneringStiffness;               % Cornering stiffness of front tires             (N/rad)
            egoVehDyn.Cr      = obj.Vehicle.RearTireCorneringStiffness;                % Cornering stiffness of rear tires              (N/rad)

            % Assign vehicle dynamics modeling parameter to base work space
            assignin('base','egoVehDyn',egoVehDyn);

            %%
            % Goal for collision mitigation >= 90%
            safetyGoal = 90;
            assignin('base','safetyGoal', safetyGoal);

            assignin('base','refPathSize', obj.RefPathSize);
            evalin('base','load("rrScenarioSimTypes.mat")');
            assignin('base','maxVelocity', 100);

            % Create driving scenario object from current scene (if
            % EnableVisualization is true) for run time visualization of
            % simulation primarily needed for road network.
            blockName = [drivingTestBenchModel, '/Scenario and Environment/Visualization/Plot Run Time Results'];
            if(obj.EnableVisualization)
                scenario = drivingScenario;
                roadNetwork(scenario,"OpenDRIVE","currentScene.xodr");
                for i=1:numActors
                    vehicle(scenario,'ClassID',actorProfiles(i).ClassID,'Position', [egoVehDyn.X0,egoVehDyn.Y0,egoVehDyn.Z0]);
                end
                assignin('base','scenario', scenario);
                set_param(blockName, "IsDisplayEnabled", 'On');
            else
                set_param(blockName, "IsDisplayEnabled", 'Off');
            end
        end

        function cleanup(~)
        % Clean up function for the DrivingTestBench.slx
        % Scenario
        %

            clearBuses({...
                'BusActionComplete',...
                'BusActorPoses',...
                'BusActorRuntime',...
                'BusLaneBoundaries',...
                'BusLaneDetections',...
                'BusPathPointData',...
                'BusLaneSensor',...
                'LaneBus',...
                'slBus1_LaneBoundaries',...
                'BusDiagnostics',...
                'BusObjectDetections1Detections',...
                'BusVehicleLocationOnLane',...
                'BusVehicleMapLocation',...
                'BusVehicleRuntime',...
                'BusDetectionConcatenation1',...
                'BusDetectionConcatenation1Detections',...
                'BusDetectionConcatenation1DetectionsMeasurementParameters',...
                'BusEgoRefPath',...
                'BusMultiObjectTracker1',...
                'BusMultiObjectTracker1Tracks',...
                'BusRadar',...
                'BusRefPath',...
                'BusRadarDetections',...
                'BusRadarDetectionsMeasurementParameters',...
                'BusRadarDetectionsObjectAttributes',...
                'BusVehiclePose',...
                'BusVision',...
                'BusVisionDetections',...
                'BusVisionDetectionsMeasurementParameters',...
                'BusVisionDetectionsObjectAttributes',...
                'BusActorPose',...
                'BusPathPointTiming',...
                'BusPathTarget',...
                'BusDeviations',...
                'BusController',...
                'BusRefPathInfo',...
                'BusVehicleCommands',...
                'LaneSensorBoundaries',...
                'LaneSensor'});
            evalin('base','clear(''scenario'')');
            evalin('base','clear(''dtleThreshold'')');
            evalin('base','clear(''LaneWidth'')');
            evalin('base','clear(''accStandstillGap'')');
            evalin('base','clear(''accTimeHeadway'')');
            evalin('base','clear(''accGapGain'')');
            evalin('base','clear(''accKp'')');
            evalin('base','clear(''accKi'')');
            evalin('base','clear(''accMinAcceleration'')');
            evalin('base','clear(''accMaxAcceleration'')');
            evalin('base','clear(''accLaneWidth'')');
            evalin('base','clear(''AccelSteerBus'')');
            evalin('base','clear(''lkaEnable'')');
            evalin('base','clear(''ControllerStepSize'')');
            evalin('base','clear(''testStartPosition'')');
            evalin('base','clear(''sensorSim'')');
            evalin('base','clear(''pathToDrivingApplication'')');
            evalin('base','clear(''visionSensorBlkPath'')');
            evalin('base','clear(''refPathSize'')');
            evalin('base','clear(''testStartTime'')');
            evalin('base','clear(''predictionHorizon'')');
            evalin('base','clear(''discretizationDistance'')');
            evalin('base','clear(''egoInitialPose'')');
            evalin('base','clear(''maxNumActors'')');
            evalin('base','clear(''numTargetActors'')');
            evalin('base','clear(''actorProfiles'')');
            evalin('base','clear(''AEB'')');
            evalin('base','clear(''cameraParams'')');
            evalin('base','clear(''Cf'')');
            evalin('base','clear(''Cr'')');
            evalin('base','clear(''egoActorID'')');
            evalin('base','clear(''egoVehDyn'')');
            evalin('base','clear(''FCW'')');
            evalin('base','clear(''vehicle'')');
            evalin('base','clear(''maxAcc'')');
            evalin('base','clear(''maxDecel'')');
            evalin('base','clear(''maxSteer'')');
            evalin('base','clear(''minAcc'')');
            evalin('base','clear(''minSteer'')');
            evalin('base','clear(''controlHorizon'')');
            evalin('base','clear(''defaultSpacing'')');
            evalin('base','clear(''timeGap'')');
            evalin('base','clear(''radarParams'')');
            evalin('base','clear(''tau'')');
            evalin('base','clear(''tau2'')');
            evalin('base','clear(''trackingParams'')');
            evalin('base','clear(''safetyGoal'')');
            evalin('base','clear(''Ts'')');
            evalin('base','clear(''steadyStateSpeed'')');
            evalin('base','clear(''maxVelocity'')');

            function clearBuses(buses)
                matlabshared.tracking.internal.DynamicBusUtilities.removeDefinition(buses);
            end
        end
    end
    methods(Static,Hidden)
        function stopSimulation(stop)
            if(stop == 1)
                rrSim = Simulink.ScenarioSimulation.find('ScenarioSimulation');
                rrSim.set('SimulationCommand','Stop');
            end
        end

        % Function to check if the driving Application is on MATLAB path
        function result = isdrivingApplicationOnPath(~)
            if(isempty(which('drivingApplication.internal.path.PathActionAdapter')))
                result = false;
            else
                result = true;
            end
        end
    end
    methods(Hidden)
        %% Vehicle dynamics parameters from scenario
        function egoVehDyn = egoVehicleDynamicsParams(obj, egoPose, egoActorProfile)
        % Ego pose for vehicle dynamics from RoadRunner Scenario.
            egoVehDyn.X0  =  egoPose.Position(1); % (m)
            egoVehDyn.Y0  = -egoPose.Position(2); % (m)
            egoVehDyn.Z0  = -egoPose.Position(3); % (m)
            egoVehDyn.VX0 =  egoPose.Velocity(1); % (m/s)
            egoVehDyn.VY0 = -egoPose.Velocity(2); % (m/s)
            egoVehDyn.VZ0 = egoPose.Velocity(3); % (m/s)

            % Adjust sign and unit of yaw
            egoVehDyn.Yaw0 = -deg2rad(egoPose.Yaw); % (rad)

            % Longitudinal velocity
            egoVehDyn.VLong0 = egoPose.vehicleFrameVelocity(1);
            egoVehDyn.VLat0 = egoPose.vehicleFrameVelocity(2);
            egoVehDyn.VVert0 = egoPose.vehicleFrameVelocity(3);

            % Distance from center of gravity to axles
            egoVehDyn.CGToFrontAxle = egoActorProfile.Length/2 - egoActorProfile.FrontOverhang;
            egoVehDyn.CGToRearAxle  = egoActorProfile.Length/2 - egoActorProfile.RearOverhang;

            % Set initial gear of the vehicle based on initial speed.
            if(egoVehDyn.VLong0<0.8)
                egoVehDyn.InitialGear = 0;
            elseif (egoVehDyn.VLong0>=0.8 && egoVehDyn.VLong0<5)
                egoVehDyn.InitialGear = 1;
            elseif (egoVehDyn.VLong0>=5 && egoVehDyn.VLong0<10)
                egoVehDyn.InitialGear = 2;
            elseif (egoVehDyn.VLong0>=10 && egoVehDyn.VLong0<15)
                egoVehDyn.InitialGear = 3;
            elseif (egoVehDyn.VLong0>=15 && egoVehDyn.VLong0<20)
                egoVehDyn.InitialGear = 4;
            elseif (egoVehDyn.VLong0>=20 && egoVehDyn.VLong0<25)
                egoVehDyn.InitialGear = 5;
            elseif (egoVehDyn.VLong0>=25 && egoVehDyn.VLong0<30)
                egoVehDyn.InitialGear = 6;
            elseif (egoVehDyn.VLong0>=30 && egoVehDyn.VLong0<35)
                egoVehDyn.InitialGear = 7;
            elseif (egoVehDyn.VLong0>=35)
                egoVehDyn.InitialGear = 8;
            end

            % Open the data dictionary.
            vehDictionaryObj = Simulink.data.dictionary.open('vehDyn14DOF.sldd');
            dDataSectObj = getSection(vehDictionaryObj,'Design Data');
            vehObj = getEntry(dDataSectObj,'VEH');
            veh = vehObj.getValue;

            % Save the dictionary only if the changes to vehicle parameters
            % are detected.
            if(veh.Mass ~= obj.Vehicle.Mass || veh.FrontAxlePositionfromCG ~= egoVehDyn.CGToFrontAxle ...
               || veh.RearAxlePositionfromCG ~= egoVehDyn.CGToRearAxle || veh.YawMomentInertia ~= obj.Vehicle.YawMomentOfInertia)
                veh.Mass = obj.Vehicle.Mass;
                veh.FrontAxlePositionfromCG = egoVehDyn.CGToFrontAxle;
                veh.RearAxlePositionfromCG = egoVehDyn.CGToRearAxle;
                veh.YawMomentInertia = obj.Vehicle.YawMomentOfInertia;
                setValue(vehObj,veh);
                saveChanges(vehDictionaryObj);
            end

            % Close the data dictionary.
            close(vehDictionaryObj);
        end


        function actorPose = getActorPose(obj, worldActors, egoSetSpeed)
        % getActorPose calculates actor pose of the actor corresponding to
        % ego actor using scenario information from RoadRunner Scenario. It
        % takes actor speed as an optional input which is used to update velocity
        % in the actor pose.

        % Find actor with ego actor id
            actorIDs = arrayfun(@(actors) str2double(actors.actor_runtime.id),worldActors,'UniformOutput',true);
            idx = actorIDs == obj.EgoActorID;
            if nnz(idx) == 1
                % Get pose matrix.
                m = worldActors(idx).actor_runtime.pose.matrix;
                c1 = m.col0;
                c2 = m.col1;
                c3 = m.col2;
                c4 = m.col3;

                pose = [c1.x c2.x c3.x c4.x; ...
                        c1.y c2.y c3.y c4.y; ...
                        c1.z c2.z c3.z c4.z; ...
                        c1.w c2.w c3.w c4.w];

                position = pose(1:3,4)';
                heading = robotics.internal.rotm2eul(pose(1:3, 1:3),'ZYX'); % The default order for Euler angle rotations is "ZYX"

                % Adjust yaw due to difference in the actor's starting orientation
                yaw = rad2deg(heading(1))+90;
                if yaw > 180
                    yaw = yaw-360;
                end
                % Actor speed is available
                actorSpeed = egoSetSpeed;
                velocityGlobal = [actorSpeed*cosd(yaw) actorSpeed*sind(yaw) 0];
                rotationMatrix = pose(1:3, 1:3);
                % Transform global velocity to vehicle-fixed frame
                velocity = (rotationMatrix' * velocityGlobal')';
                actorPose = struct(...
                    'ActorID', double(obj.EgoActorID), ...
                    'Position', position, ...         % m
                    'Velocity', velocityGlobal, ...         % m/s
                    'vehicleFrameVelocity', [velocity(2), -velocity(1), velocity(3)],...
                    'Roll', rad2deg(heading(3)), ...  % deg
                    'Pitch', rad2deg(heading(2)), ... % deg
                    'Yaw', yaw, ...                   % deg
                    'AngularVelocity', [0 0 0]);      % deg/s
            else
                error("Actor with ActorID:%d does not exist", obj.EgoActorID);
            end
        end

        function [actorProfiles, actorIdToIndexMap] = getActorProfiles(~, worldActors, originOffsetReference)
            % getActorProfiles calculates driving scenario actor profiles
            % for all the actors using actor spec information from RoadRunner
            % Scenario.
            %
            % getActorProfiles take worldActors and OriginOffset as input.
            % By default, the origin offset is set to 'VehicleCenter' and
            % returns the origin offset of the actor with respect to the
            % vehicle center. If OriginOffset is set to  'RearAxleCenter'
            % then the vehicle's origin offset will be calculated as the
            % offset from the real axle center to the geometric center of
            % the vehicle.
            %
            % Examples of calling this function:
            %   [actorProfiles, actorIdToIndexMap] = getActorProfiles(worldActors,"OriginOffset","RearAxleCenter")

            numActors = length(worldActors);

            % Initialize actorProfiles struct.
            actorProfile = struct(...
                'ActorID',0,...
                'ClassID',1,...
                'Length',0,...
                'Width',0,...
                'Height',0,...
                'OriginOffset',[0 0 0],...
                'FrontOverhang',0,...
                'RearOverhang',0,...
                'Color',[0 0 0],...
                'bbx',zeros(2,3));

            actorProfiles = repmat(actorProfile, 1, numActors);

            % Create a map from ActorID to index
            actorIdToIndexMap = containers.Map('KeyType', 'double', 'ValueType', 'double');

            for i = 1:numActors
                id = str2double(worldActors(i).actor_spec.id);
                
                % Store the mapping of ActorID to index
                actorIdToIndexMap(id) = i;

                % Get actor bounding box
                min = worldActors(i).actor_spec.bounding_box.min;
                max = worldActors(i).actor_spec.bounding_box.max;
                bbx = [min.x min.y min.z; ...
                       max.x max.y max.z];
                actorProfiles(i).ActorID = id;

                % Calculate length and width from bounding boxes
                actorProfiles(i).Length  = max.y - min.y;
                actorProfiles(i).Width   = max.x - min.x;
                actorProfiles(i).Height  = max.z;
                actorProfiles(i).bbx     = bbx;


                % Update the Color, FrontOverhang, RearOverhang and OriginOffset values
                % in the actor profiles if it is a vehicle. For character, these values
                % are not valid.
                if ~isempty(worldActors(i).actor_spec.vehicle_spec) && isempty(worldActors(i).actor_spec.character_spec)
                    % Get actor color
                    color = worldActors(i).actor_spec.vehicle_spec.paint_color;
                    r = double(color.r)/255;
                    g = double(color.g)/255;
                    b = double(color.b)/255;
                    actorProfiles(i).Color   = [r g b];
                    if(~isempty(worldActors(i).actor_spec.vehicle_spec.wheels))
                        % Calculate FrontOverhang and RearOverhang
                        actorProfiles(i).FrontOverhang = actorProfiles(i).Length/2 - worldActors(i).actor_spec.vehicle_spec.wheels(1).wheel_offset.y;
                        actorProfiles(i).RearOverhang = actorProfiles(i).Length/2 + worldActors(i).actor_spec.vehicle_spec.wheels(3).wheel_offset.y;
                    else % For foam car there is no wheel spec availble from RR
                        actorProfiles(i).FrontOverhang = 0.78;
                        actorProfiles(i).RearOverhang = 0.68;
                    end
                    %Calculate OriginOffset
                    if(originOffsetReference == "RearAxleCenter")
                        actorProfiles(i).OriginOffset(1) = actorProfiles(i).RearOverhang - actorProfiles(i).Length/2 ;
                    end
                end

                if(~isempty(worldActors(i).actor_spec.character_spec))
                    actorProfiles(i).ClassID = 4; % Update Class ID for pedestrian actor.
                end
            end
        end

        % createBusObjects creates bus objects used by DrivingTestBench.slx.
        function createBusObjects(obj)
            % Bus object: BusActorPoses
            BusActorPoses = Simulink.Bus;
            BusActorPoses.Description = '';
            BusActorPoses.DataScope = 'Auto';
            BusActorPoses.HeaderFile = '';
            BusActorPoses.Alignment = -1;
            saveVarsTmp{1} = Simulink.BusElement;
            saveVarsTmp{1}.Name = 'NumActors';
            saveVarsTmp{1}.Complexity = 'real';
            saveVarsTmp{1}.Dimensions = [1 1];
            saveVarsTmp{1}.DataType = 'double';
            saveVarsTmp{1}.Min = [];
            saveVarsTmp{1}.Max = [];
            saveVarsTmp{1}.DimensionsMode = 'Fixed';
            saveVarsTmp{1}.SamplingMode = 'Sample based';
            saveVarsTmp{1}.DocUnits = '';
            saveVarsTmp{1}.Description = '';
            saveVarsTmp{1}(2, 1) = Simulink.BusElement;
            saveVarsTmp{1}(2, 1).Name = 'Time';
            saveVarsTmp{1}(2, 1).Complexity = 'real';
            saveVarsTmp{1}(2, 1).Dimensions = [1 1];
            saveVarsTmp{1}(2, 1).DataType = 'double';
            saveVarsTmp{1}(2, 1).Min = [];
            saveVarsTmp{1}(2, 1).Max = [];
            saveVarsTmp{1}(2, 1).DimensionsMode = 'Fixed';
            saveVarsTmp{1}(2, 1).SamplingMode = 'Sample based';
            saveVarsTmp{1}(2, 1).DocUnits = '';
            saveVarsTmp{1}(2, 1).Description = '';
            saveVarsTmp{1}(3, 1) = Simulink.BusElement;
            saveVarsTmp{1}(3, 1).Name = 'Actors';
            saveVarsTmp{1}(3, 1).Complexity = 'real';
            saveVarsTmp{1}(3, 1).Dimensions = [obj.NumTargetActors+1 1];
            saveVarsTmp{1}(3, 1).DataType = 'Bus: BusVehiclePose';
            saveVarsTmp{1}(3, 1).Min = [];
            saveVarsTmp{1}(3, 1).Max = [];
            saveVarsTmp{1}(3, 1).DimensionsMode = 'Fixed';
            saveVarsTmp{1}(3, 1).SamplingMode = 'Sample based';
            saveVarsTmp{1}(3, 1).DocUnits = '';
            saveVarsTmp{1}(3, 1).Description = '';
            BusActorPoses.Elements = saveVarsTmp{1};
            clear saveVarsTmp;

            % Bus object: BusVehiclePose
            BusVehiclePose = Simulink.Bus;
            BusVehiclePose.Description = '';
            BusVehiclePose.DataScope = 'Auto';
            BusVehiclePose.HeaderFile = '';
            BusVehiclePose.Alignment = -1;
            saveVarsTmp{1} = Simulink.BusElement;
            saveVarsTmp{1}.Name = 'ActorID';
            saveVarsTmp{1}.Complexity = 'real';
            saveVarsTmp{1}.Dimensions = [1 1];
            saveVarsTmp{1}.DataType = 'double';
            saveVarsTmp{1}.Min = [];
            saveVarsTmp{1}.Max = [];
            saveVarsTmp{1}.DimensionsMode = 'Fixed';
            saveVarsTmp{1}.SamplingMode = 'Sample based';
            saveVarsTmp{1}.DocUnits = '';
            saveVarsTmp{1}.Description = '';
            saveVarsTmp{1}(2, 1) = Simulink.BusElement;
            saveVarsTmp{1}(2, 1).Name = 'Position';
            saveVarsTmp{1}(2, 1).Complexity = 'real';
            saveVarsTmp{1}(2, 1).Dimensions = [1 3];
            saveVarsTmp{1}(2, 1).DataType = 'double';
            saveVarsTmp{1}(2, 1).Min = [];
            saveVarsTmp{1}(2, 1).Max = [];
            saveVarsTmp{1}(2, 1).DimensionsMode = 'Fixed';
            saveVarsTmp{1}(2, 1).SamplingMode = 'Sample based';
            saveVarsTmp{1}(2, 1).DocUnits = '';
            saveVarsTmp{1}(2, 1).Description = '';
            saveVarsTmp{1}(3, 1) = Simulink.BusElement;
            saveVarsTmp{1}(3, 1).Name = 'Velocity';
            saveVarsTmp{1}(3, 1).Complexity = 'real';
            saveVarsTmp{1}(3, 1).Dimensions = [1 3];
            saveVarsTmp{1}(3, 1).DataType = 'double';
            saveVarsTmp{1}(3, 1).Min = [];
            saveVarsTmp{1}(3, 1).Max = [];
            saveVarsTmp{1}(3, 1).DimensionsMode = 'Fixed';
            saveVarsTmp{1}(3, 1).SamplingMode = 'Sample based';
            saveVarsTmp{1}(3, 1).DocUnits = '';
            saveVarsTmp{1}(3, 1).Description = '';
            saveVarsTmp{1}(4, 1) = Simulink.BusElement;
            saveVarsTmp{1}(4, 1).Name = 'Roll';
            saveVarsTmp{1}(4, 1).Complexity = 'real';
            saveVarsTmp{1}(4, 1).Dimensions = [1 1];
            saveVarsTmp{1}(4, 1).DataType = 'double';
            saveVarsTmp{1}(4, 1).Min = [];
            saveVarsTmp{1}(4, 1).Max = [];
            saveVarsTmp{1}(4, 1).DimensionsMode = 'Fixed';
            saveVarsTmp{1}(4, 1).SamplingMode = 'Sample based';
            saveVarsTmp{1}(4, 1).DocUnits = '';
            saveVarsTmp{1}(4, 1).Description = '';
            saveVarsTmp{1}(5, 1) = Simulink.BusElement;
            saveVarsTmp{1}(5, 1).Name = 'Pitch';
            saveVarsTmp{1}(5, 1).Complexity = 'real';
            saveVarsTmp{1}(5, 1).Dimensions = [1 1];
            saveVarsTmp{1}(5, 1).DataType = 'double';
            saveVarsTmp{1}(5, 1).Min = [];
            saveVarsTmp{1}(5, 1).Max = [];
            saveVarsTmp{1}(5, 1).DimensionsMode = 'Fixed';
            saveVarsTmp{1}(5, 1).SamplingMode = 'Sample based';
            saveVarsTmp{1}(5, 1).DocUnits = '';
            saveVarsTmp{1}(5, 1).Description = '';
            saveVarsTmp{1}(6, 1) = Simulink.BusElement;
            saveVarsTmp{1}(6, 1).Name = 'Yaw';
            saveVarsTmp{1}(6, 1).Complexity = 'real';
            saveVarsTmp{1}(6, 1).Dimensions = [1 1];
            saveVarsTmp{1}(6, 1).DataType = 'double';
            saveVarsTmp{1}(6, 1).Min = [];
            saveVarsTmp{1}(6, 1).Max = [];
            saveVarsTmp{1}(6, 1).DimensionsMode = 'Fixed';
            saveVarsTmp{1}(6, 1).SamplingMode = 'Sample based';
            saveVarsTmp{1}(6, 1).DocUnits = '';
            saveVarsTmp{1}(6, 1).Description = '';
            saveVarsTmp{1}(7, 1) = Simulink.BusElement;
            saveVarsTmp{1}(7, 1).Name = 'AngularVelocity';
            saveVarsTmp{1}(7, 1).Complexity = 'real';
            saveVarsTmp{1}(7, 1).Dimensions = [1 3];
            saveVarsTmp{1}(7, 1).DataType = 'double';
            saveVarsTmp{1}(7, 1).Min = [];
            saveVarsTmp{1}(7, 1).Max = [];
            saveVarsTmp{1}(7, 1).DimensionsMode = 'Fixed';
            saveVarsTmp{1}(7, 1).SamplingMode = 'Sample based';
            saveVarsTmp{1}(7, 1).DocUnits = '';
            saveVarsTmp{1}(7, 1).Description = '';
            BusVehiclePose.Elements = saveVarsTmp{1};
            clear saveVarsTmp;

            % Bus object: BusEgoRefPath
            BusEgoRefPath = Simulink.Bus;
            BusEgoRefPath.Description = '';
            BusEgoRefPath.DataScope = 'Auto';
            BusEgoRefPath.HeaderFile = '';
            BusEgoRefPath.Alignment = -1;
            BusEgoRefPath.PreserveElementDimensions = false;
            saveVarsTmp{1}(1, 1) = Simulink.BusElement;
            saveVarsTmp{1}(1, 1).Name = 'x';
            saveVarsTmp{1}(1, 1).Complexity = 'real';
            saveVarsTmp{1}(1, 1).Dimensions = obj.RefPathSize;
            saveVarsTmp{1}(1, 1).DataType = 'double';
            saveVarsTmp{1}(1, 1).Min = [];
            saveVarsTmp{1}(1, 1).Max = [];
            saveVarsTmp{1}(1, 1).DimensionsMode = 'Fixed';
            saveVarsTmp{1}(1, 1).SamplingMode = 'Sample based';
            saveVarsTmp{1}(1, 1).DocUnits = '';
            saveVarsTmp{1}(1, 1).Description = '';
            saveVarsTmp{1}(2, 1) = Simulink.BusElement;
            saveVarsTmp{1}(2, 1).Name = 'y';
            saveVarsTmp{1}(2, 1).Complexity = 'real';
            saveVarsTmp{1}(2, 1).Dimensions = obj.RefPathSize;
            saveVarsTmp{1}(2, 1).DataType = 'double';
            saveVarsTmp{1}(2, 1).Min = [];
            saveVarsTmp{1}(2, 1).Max = [];
            saveVarsTmp{1}(2, 1).DimensionsMode = 'Fixed';
            saveVarsTmp{1}(2, 1).SamplingMode = 'Sample based';
            saveVarsTmp{1}(2, 1).DocUnits = '';
            saveVarsTmp{1}(2, 1).Description = '';
            saveVarsTmp{1}(3, 1) = Simulink.BusElement;
            saveVarsTmp{1}(3, 1).Name = 'theta';
            saveVarsTmp{1}(3, 1).Complexity = 'real';
            saveVarsTmp{1}(3, 1).Dimensions = obj.RefPathSize;
            saveVarsTmp{1}(3, 1).DataType = 'double';
            saveVarsTmp{1}(3, 1).Min = [];
            saveVarsTmp{1}(3, 1).Max = [];
            saveVarsTmp{1}(3, 1).DimensionsMode = 'Fixed';
            saveVarsTmp{1}(3, 1).SamplingMode = 'Sample based';
            saveVarsTmp{1}(3, 1).DocUnits = '';
            saveVarsTmp{1}(3, 1).Description = '';
            saveVarsTmp{1}(4, 1) = Simulink.BusElement;
            saveVarsTmp{1}(4, 1).Name = 'kappa';
            saveVarsTmp{1}(4, 1).Complexity = 'real';
            saveVarsTmp{1}(4, 1).Dimensions = obj.RefPathSize;
            saveVarsTmp{1}(4, 1).DataType = 'double';
            saveVarsTmp{1}(4, 1).Min = [];
            saveVarsTmp{1}(4, 1).Max = [];
            saveVarsTmp{1}(4, 1).DimensionsMode = 'Fixed';
            saveVarsTmp{1}(4, 1).SamplingMode = 'Sample based';
            saveVarsTmp{1}(4, 1).DocUnits = '';
            saveVarsTmp{1}(4, 1).Description = '';
            saveVarsTmp{1}(5, 1) = Simulink.BusElement;
            saveVarsTmp{1}(5, 1).Name = 'speed';
            saveVarsTmp{1}(5, 1).Complexity = 'real';
            saveVarsTmp{1}(5, 1).Dimensions = obj.RefPathSize;
            saveVarsTmp{1}(5, 1).DataType = 'double';
            saveVarsTmp{1}(5, 1).Min = [];
            saveVarsTmp{1}(5, 1).Max = [];
            saveVarsTmp{1}(5, 1).DimensionsMode = 'Fixed';
            saveVarsTmp{1}(5, 1).SamplingMode = 'Sample based';
            saveVarsTmp{1}(5, 1).DocUnits = '';
            saveVarsTmp{1}(5, 1).Description = '';
            saveVarsTmp{1}(6, 1) = Simulink.BusElement;
            saveVarsTmp{1}(6, 1).Name = 'arcLength';
            saveVarsTmp{1}(6, 1).Complexity = 'real';
            saveVarsTmp{1}(6, 1).Dimensions = obj.RefPathSize;
            saveVarsTmp{1}(6, 1).DataType = 'double';
            saveVarsTmp{1}(6, 1).Min = [];
            saveVarsTmp{1}(6, 1).Max = [];
            saveVarsTmp{1}(6, 1).DimensionsMode = 'Fixed';
            saveVarsTmp{1}(6, 1).SamplingMode = 'Sample based';
            saveVarsTmp{1}(6, 1).DocUnits = '';
            saveVarsTmp{1}(6, 1).Description = '';
            saveVarsTmp{1}(7, 1).Name = 'numPoints';
            saveVarsTmp{1}(7, 1).Complexity = 'real';
            saveVarsTmp{1}(7, 1).Dimensions = [1, 1];
            saveVarsTmp{1}(7, 1).DataType = 'double';
            saveVarsTmp{1}(7, 1).Min = [];
            saveVarsTmp{1}(7, 1).Max = [];
            saveVarsTmp{1}(7, 1).DimensionsMode = 'Fixed';
            saveVarsTmp{1}(7, 1).SamplingMode = 'Sample based';
            saveVarsTmp{1}(7, 1).DocUnits = '';
            saveVarsTmp{1}(7, 1).Description = '';
            BusEgoRefPath.Elements = saveVarsTmp{1};
            clear saveVarsTmp;

            % Create BusRadar
            BusRadar = Simulink.Bus;
            BusRadar.Description = '';
            BusRadar.DataScope = 'Auto';
            BusRadar.HeaderFile = '';
            BusRadar.Alignment = -1;
            BusRadar.PreserveElementDimensions = false;
            saveVarsTmp{1} = Simulink.BusElement;
            saveVarsTmp{1}.Name = 'NumDetections';
            saveVarsTmp{1}.Complexity = 'real';
            saveVarsTmp{1}.Dimensions = [1 1];
            saveVarsTmp{1}.DataType = 'double';
            saveVarsTmp{1}.Min = [];
            saveVarsTmp{1}.Max = [];
            saveVarsTmp{1}.DimensionsMode = 'Fixed';
            saveVarsTmp{1}.SamplingMode = 'Sample based';
            saveVarsTmp{1}.DocUnits = '';
            saveVarsTmp{1}.Description = '';
            saveVarsTmp{1}(2, 1) = Simulink.BusElement;
            saveVarsTmp{1}(2, 1).Name = 'IsValidTime';
            saveVarsTmp{1}(2, 1).Complexity = 'real';
            saveVarsTmp{1}(2, 1).Dimensions = [1 1];
            saveVarsTmp{1}(2, 1).DataType = 'boolean';
            saveVarsTmp{1}(2, 1).Min = [];
            saveVarsTmp{1}(2, 1).Max = [];
            saveVarsTmp{1}(2, 1).DimensionsMode = 'Fixed';
            saveVarsTmp{1}(2, 1).SamplingMode = 'Sample based';
            saveVarsTmp{1}(2, 1).DocUnits = '';
            saveVarsTmp{1}(2, 1).Description = '';
            saveVarsTmp{1}(3, 1) = Simulink.BusElement;
            saveVarsTmp{1}(3, 1).Name = 'Detections';
            saveVarsTmp{1}(3, 1).Complexity = 'real';
            saveVarsTmp{1}(3, 1).Dimensions = [50 1];
            saveVarsTmp{1}(3, 1).DataType = 'Bus: BusRadarDetections';
            saveVarsTmp{1}(3, 1).Min = [];
            saveVarsTmp{1}(3, 1).Max = [];
            saveVarsTmp{1}(3, 1).DimensionsMode = 'Fixed';
            saveVarsTmp{1}(3, 1).SamplingMode = 'Sample based';
            saveVarsTmp{1}(3, 1).DocUnits = '';
            saveVarsTmp{1}(3, 1).Description = '';
            BusRadar.Elements = saveVarsTmp{1};
            clear saveVarsTmp;

            % Create BusRadarDetections
            BusRadarDetections = Simulink.Bus;
            BusRadarDetections.Description = '';
            BusRadarDetections.DataScope = 'Auto';
            BusRadarDetections.HeaderFile = '';
            BusRadarDetections.Alignment = -1;
            BusRadarDetections.PreserveElementDimensions = false;
            saveVarsTmp{1} = Simulink.BusElement;
            saveVarsTmp{1}.Name = 'Time';
            saveVarsTmp{1}.Complexity = 'real';
            saveVarsTmp{1}.Dimensions = [1 1];
            saveVarsTmp{1}.DataType = 'double';
            saveVarsTmp{1}.Min = [];
            saveVarsTmp{1}.Max = [];
            saveVarsTmp{1}.DimensionsMode = 'Fixed';
            saveVarsTmp{1}.SamplingMode = 'Sample based';
            saveVarsTmp{1}.DocUnits = '';
            saveVarsTmp{1}.Description = '';
            saveVarsTmp{1}(2, 1) = Simulink.BusElement;
            saveVarsTmp{1}(2, 1).Name = 'Measurement';
            saveVarsTmp{1}(2, 1).Complexity = 'real';
            saveVarsTmp{1}(2, 1).Dimensions = [6 1];
            saveVarsTmp{1}(2, 1).DataType = 'double';
            saveVarsTmp{1}(2, 1).Min = [];
            saveVarsTmp{1}(2, 1).Max = [];
            saveVarsTmp{1}(2, 1).DimensionsMode = 'Fixed';
            saveVarsTmp{1}(2, 1).SamplingMode = 'Sample based';
            saveVarsTmp{1}(2, 1).DocUnits = '';
            saveVarsTmp{1}(2, 1).Description = '';
            saveVarsTmp{1}(3, 1) = Simulink.BusElement;
            saveVarsTmp{1}(3, 1).Name = 'MeasurementNoise';
            saveVarsTmp{1}(3, 1).Complexity = 'real';
            saveVarsTmp{1}(3, 1).Dimensions = [6 6];
            saveVarsTmp{1}(3, 1).DataType = 'double';
            saveVarsTmp{1}(3, 1).Min = [];
            saveVarsTmp{1}(3, 1).Max = [];
            saveVarsTmp{1}(3, 1).DimensionsMode = 'Fixed';
            saveVarsTmp{1}(3, 1).SamplingMode = 'Sample based';
            saveVarsTmp{1}(3, 1).DocUnits = '';
            saveVarsTmp{1}(3, 1).Description = '';
            saveVarsTmp{1}(4, 1) = Simulink.BusElement;
            saveVarsTmp{1}(4, 1).Name = 'SensorIndex';
            saveVarsTmp{1}(4, 1).Complexity = 'real';
            saveVarsTmp{1}(4, 1).Dimensions = [1 1];
            saveVarsTmp{1}(4, 1).DataType = 'double';
            saveVarsTmp{1}(4, 1).Min = [];
            saveVarsTmp{1}(4, 1).Max = [];
            saveVarsTmp{1}(4, 1).DimensionsMode = 'Fixed';
            saveVarsTmp{1}(4, 1).SamplingMode = 'Sample based';
            saveVarsTmp{1}(4, 1).DocUnits = '';
            saveVarsTmp{1}(4, 1).Description = '';
            saveVarsTmp{1}(5, 1) = Simulink.BusElement;
            saveVarsTmp{1}(5, 1).Name = 'ObjectClassID';
            saveVarsTmp{1}(5, 1).Complexity = 'real';
            saveVarsTmp{1}(5, 1).Dimensions = [1 1];
            saveVarsTmp{1}(5, 1).DataType = 'double';
            saveVarsTmp{1}(5, 1).Min = [];
            saveVarsTmp{1}(5, 1).Max = [];
            saveVarsTmp{1}(5, 1).DimensionsMode = 'Fixed';
            saveVarsTmp{1}(5, 1).SamplingMode = 'Sample based';
            saveVarsTmp{1}(5, 1).DocUnits = '';
            saveVarsTmp{1}(5, 1).Description = '';
            saveVarsTmp{1}(6, 1) = Simulink.BusElement;
            saveVarsTmp{1}(6, 1).Name = 'MeasurementParameters';
            saveVarsTmp{1}(6, 1).Complexity = 'real';
            saveVarsTmp{1}(6, 1).Dimensions = [1 1];
            saveVarsTmp{1}(6, 1).DataType = ['Bus: BusRadarDetectionsMeasurementPar' ...
                                             'ameters'];
            saveVarsTmp{1}(6, 1).Min = [];
            saveVarsTmp{1}(6, 1).Max = [];
            saveVarsTmp{1}(6, 1).DimensionsMode = 'Fixed';
            saveVarsTmp{1}(6, 1).SamplingMode = 'Sample based';
            saveVarsTmp{1}(6, 1).DocUnits = '';
            saveVarsTmp{1}(6, 1).Description = '';
            saveVarsTmp{1}(7, 1) = Simulink.BusElement;
            saveVarsTmp{1}(7, 1).Name = 'ObjectAttributes';
            saveVarsTmp{1}(7, 1).Complexity = 'real';
            saveVarsTmp{1}(7, 1).Dimensions = [1 1];
            saveVarsTmp{1}(7, 1).DataType = ['Bus: BusRadarDetectionsObjectAttribut' ...
                                             'es'];
            saveVarsTmp{1}(7, 1).Min = [];
            saveVarsTmp{1}(7, 1).Max = [];
            saveVarsTmp{1}(7, 1).DimensionsMode = 'Fixed';
            saveVarsTmp{1}(7, 1).SamplingMode = 'Sample based';
            saveVarsTmp{1}(7, 1).DocUnits = '';
            saveVarsTmp{1}(7, 1).Description = '';
            BusRadarDetections.Elements = saveVarsTmp{1};
            clear saveVarsTmp;

            % Create BusRadarDetectionsMeasurementParameters
            BusRadarDetectionsMeasurementParameters = Simulink.Bus;
            BusRadarDetectionsMeasurementParameters.Description = '';
            BusRadarDetectionsMeasurementParameters.DataScope = 'Auto';
            BusRadarDetectionsMeasurementParameters.HeaderFile = '';
            BusRadarDetectionsMeasurementParameters.Alignment = -1;
            BusRadarDetectionsMeasurementParameters.PreserveElementDimensions = false;
            saveVarsTmp{1} = Simulink.BusElement;
            saveVarsTmp{1}.Name = 'Frame';
            saveVarsTmp{1}.Complexity = 'real';
            saveVarsTmp{1}.Dimensions = [1 1];
            saveVarsTmp{1}.DataType = 'Enum: drivingCoordinateFrameType';
            saveVarsTmp{1}.Min = [];
            saveVarsTmp{1}.Max = [];
            saveVarsTmp{1}.DimensionsMode = 'Fixed';
            saveVarsTmp{1}.SamplingMode = 'Sample based';
            saveVarsTmp{1}.DocUnits = '';
            saveVarsTmp{1}.Description = '';
            saveVarsTmp{1}(2, 1) = Simulink.BusElement;
            saveVarsTmp{1}(2, 1).Name = 'OriginPosition';
            saveVarsTmp{1}(2, 1).Complexity = 'real';
            saveVarsTmp{1}(2, 1).Dimensions = [3 1];
            saveVarsTmp{1}(2, 1).DataType = 'double';
            saveVarsTmp{1}(2, 1).Min = [];
            saveVarsTmp{1}(2, 1).Max = [];
            saveVarsTmp{1}(2, 1).DimensionsMode = 'Fixed';
            saveVarsTmp{1}(2, 1).SamplingMode = 'Sample based';
            saveVarsTmp{1}(2, 1).DocUnits = '';
            saveVarsTmp{1}(2, 1).Description = '';
            saveVarsTmp{1}(3, 1) = Simulink.BusElement;
            saveVarsTmp{1}(3, 1).Name = 'OriginVelocity';
            saveVarsTmp{1}(3, 1).Complexity = 'real';
            saveVarsTmp{1}(3, 1).Dimensions = [3 1];
            saveVarsTmp{1}(3, 1).DataType = 'double';
            saveVarsTmp{1}(3, 1).Min = [];
            saveVarsTmp{1}(3, 1).Max = [];
            saveVarsTmp{1}(3, 1).DimensionsMode = 'Fixed';
            saveVarsTmp{1}(3, 1).SamplingMode = 'Sample based';
            saveVarsTmp{1}(3, 1).DocUnits = '';
            saveVarsTmp{1}(3, 1).Description = '';
            saveVarsTmp{1}(4, 1) = Simulink.BusElement;
            saveVarsTmp{1}(4, 1).Name = 'Orientation';
            saveVarsTmp{1}(4, 1).Complexity = 'real';
            saveVarsTmp{1}(4, 1).Dimensions = [3 3];
            saveVarsTmp{1}(4, 1).DataType = 'double';
            saveVarsTmp{1}(4, 1).Min = [];
            saveVarsTmp{1}(4, 1).Max = [];
            saveVarsTmp{1}(4, 1).DimensionsMode = 'Fixed';
            saveVarsTmp{1}(4, 1).SamplingMode = 'Sample based';
            saveVarsTmp{1}(4, 1).DocUnits = '';
            saveVarsTmp{1}(4, 1).Description = '';
            saveVarsTmp{1}(5, 1) = Simulink.BusElement;
            saveVarsTmp{1}(5, 1).Name = 'IsParentToChild';
            saveVarsTmp{1}(5, 1).Complexity = 'real';
            saveVarsTmp{1}(5, 1).Dimensions = [1 1];
            saveVarsTmp{1}(5, 1).DataType = 'boolean';
            saveVarsTmp{1}(5, 1).Min = [];
            saveVarsTmp{1}(5, 1).Max = [];
            saveVarsTmp{1}(5, 1).DimensionsMode = 'Fixed';
            saveVarsTmp{1}(5, 1).SamplingMode = 'Sample based';
            saveVarsTmp{1}(5, 1).DocUnits = '';
            saveVarsTmp{1}(5, 1).Description = '';
            saveVarsTmp{1}(6, 1) = Simulink.BusElement;
            saveVarsTmp{1}(6, 1).Name = 'HasAzimuth';
            saveVarsTmp{1}(6, 1).Complexity = 'real';
            saveVarsTmp{1}(6, 1).Dimensions = [1 1];
            saveVarsTmp{1}(6, 1).DataType = 'boolean';
            saveVarsTmp{1}(6, 1).Min = [];
            saveVarsTmp{1}(6, 1).Max = [];
            saveVarsTmp{1}(6, 1).DimensionsMode = 'Fixed';
            saveVarsTmp{1}(6, 1).SamplingMode = 'Sample based';
            saveVarsTmp{1}(6, 1).DocUnits = '';
            saveVarsTmp{1}(6, 1).Description = '';
            saveVarsTmp{1}(7, 1) = Simulink.BusElement;
            saveVarsTmp{1}(7, 1).Name = 'HasElevation';
            saveVarsTmp{1}(7, 1).Complexity = 'real';
            saveVarsTmp{1}(7, 1).Dimensions = [1 1];
            saveVarsTmp{1}(7, 1).DataType = 'boolean';
            saveVarsTmp{1}(7, 1).Min = [];
            saveVarsTmp{1}(7, 1).Max = [];
            saveVarsTmp{1}(7, 1).DimensionsMode = 'Fixed';
            saveVarsTmp{1}(7, 1).SamplingMode = 'Sample based';
            saveVarsTmp{1}(7, 1).DocUnits = '';
            saveVarsTmp{1}(7, 1).Description = '';
            saveVarsTmp{1}(8, 1) = Simulink.BusElement;
            saveVarsTmp{1}(8, 1).Name = 'HasRange';
            saveVarsTmp{1}(8, 1).Complexity = 'real';
            saveVarsTmp{1}(8, 1).Dimensions = [1 1];
            saveVarsTmp{1}(8, 1).DataType = 'boolean';
            saveVarsTmp{1}(8, 1).Min = [];
            saveVarsTmp{1}(8, 1).Max = [];
            saveVarsTmp{1}(8, 1).DimensionsMode = 'Fixed';
            saveVarsTmp{1}(8, 1).SamplingMode = 'Sample based';
            saveVarsTmp{1}(8, 1).DocUnits = '';
            saveVarsTmp{1}(8, 1).Description = '';
            saveVarsTmp{1}(9, 1) = Simulink.BusElement;
            saveVarsTmp{1}(9, 1).Name = 'HasVelocity';
            saveVarsTmp{1}(9, 1).Complexity = 'real';
            saveVarsTmp{1}(9, 1).Dimensions = [1 1];
            saveVarsTmp{1}(9, 1).DataType = 'boolean';
            saveVarsTmp{1}(9, 1).Min = [];
            saveVarsTmp{1}(9, 1).Max = [];
            saveVarsTmp{1}(9, 1).DimensionsMode = 'Fixed';
            saveVarsTmp{1}(9, 1).SamplingMode = 'Sample based';
            saveVarsTmp{1}(9, 1).DocUnits = '';
            saveVarsTmp{1}(9, 1).Description = '';
            BusRadarDetectionsMeasurementParameters.Elements = saveVarsTmp{1};
            clear saveVarsTmp;

            % Create BusRadarDetectionsObjectAttributes
            BusRadarDetectionsObjectAttributes = Simulink.Bus;
            BusRadarDetectionsObjectAttributes.Description = '';
            BusRadarDetectionsObjectAttributes.DataScope = 'Auto';
            BusRadarDetectionsObjectAttributes.HeaderFile = '';
            BusRadarDetectionsObjectAttributes.Alignment = -1;
            BusRadarDetectionsObjectAttributes.PreserveElementDimensions = false;
            saveVarsTmp{1} = Simulink.BusElement;
            saveVarsTmp{1}.Name = 'TargetIndex';
            saveVarsTmp{1}.Complexity = 'real';
            saveVarsTmp{1}.Dimensions = [1 1];
            saveVarsTmp{1}.DataType = 'double';
            saveVarsTmp{1}.Min = [];
            saveVarsTmp{1}.Max = [];
            saveVarsTmp{1}.DimensionsMode = 'Fixed';
            saveVarsTmp{1}.SamplingMode = 'Sample based';
            saveVarsTmp{1}.DocUnits = '';
            saveVarsTmp{1}.Description = '';
            saveVarsTmp{1}(2, 1) = Simulink.BusElement;
            saveVarsTmp{1}(2, 1).Name = 'SNR';
            saveVarsTmp{1}(2, 1).Complexity = 'real';
            saveVarsTmp{1}(2, 1).Dimensions = [1 1];
            saveVarsTmp{1}(2, 1).DataType = 'double';
            saveVarsTmp{1}(2, 1).Min = [];
            saveVarsTmp{1}(2, 1).Max = [];
            saveVarsTmp{1}(2, 1).DimensionsMode = 'Fixed';
            saveVarsTmp{1}(2, 1).SamplingMode = 'Sample based';
            saveVarsTmp{1}(2, 1).DocUnits = '';
            saveVarsTmp{1}(2, 1).Description = '';
            BusRadarDetectionsObjectAttributes.Elements = saveVarsTmp{1};
            clear saveVarsTmp;

            % Create BusVision
            BusVision = Simulink.Bus;
            BusVision.Description = '';
            BusVision.DataScope = 'Auto';
            BusVision.HeaderFile = '';
            BusVision.Alignment = -1;
            BusVision.PreserveElementDimensions = false;
            saveVarsTmp{1} = Simulink.BusElement;
            saveVarsTmp{1}.Name = 'NumDetections';
            saveVarsTmp{1}.Complexity = 'real';
            saveVarsTmp{1}.Dimensions = [1 1];
            saveVarsTmp{1}.DataType = 'double';
            saveVarsTmp{1}.Min = [];
            saveVarsTmp{1}.Max = [];
            saveVarsTmp{1}.DimensionsMode = 'Fixed';
            saveVarsTmp{1}.SamplingMode = 'Sample based';
            saveVarsTmp{1}.DocUnits = '';
            saveVarsTmp{1}.Description = '';
            saveVarsTmp{1}(2, 1) = Simulink.BusElement;
            saveVarsTmp{1}(2, 1).Name = 'IsValidTime';
            saveVarsTmp{1}(2, 1).Complexity = 'real';
            saveVarsTmp{1}(2, 1).Dimensions = [1 1];
            saveVarsTmp{1}(2, 1).DataType = 'boolean';
            saveVarsTmp{1}(2, 1).Min = [];
            saveVarsTmp{1}(2, 1).Max = [];
            saveVarsTmp{1}(2, 1).DimensionsMode = 'Fixed';
            saveVarsTmp{1}(2, 1).SamplingMode = 'Sample based';
            saveVarsTmp{1}(2, 1).DocUnits = '';
            saveVarsTmp{1}(2, 1).Description = '';
            saveVarsTmp{1}(3, 1) = Simulink.BusElement;
            saveVarsTmp{1}(3, 1).Name = 'Detections';
            saveVarsTmp{1}(3, 1).Complexity = 'real';
            saveVarsTmp{1}(3, 1).Dimensions = [20 1];
            saveVarsTmp{1}(3, 1).DataType = 'Bus: BusVisionDetections';
            saveVarsTmp{1}(3, 1).Min = [];
            saveVarsTmp{1}(3, 1).Max = [];
            saveVarsTmp{1}(3, 1).DimensionsMode = 'Fixed';
            saveVarsTmp{1}(3, 1).SamplingMode = 'Sample based';
            saveVarsTmp{1}(3, 1).DocUnits = '';
            saveVarsTmp{1}(3, 1).Description = '';
            BusVision.Elements = saveVarsTmp{1};
            clear saveVarsTmp;

            % Create BusVisionDetections
            BusVisionDetections = Simulink.Bus;
            BusVisionDetections.Description = '';
            BusVisionDetections.DataScope = 'Auto';
            BusVisionDetections.HeaderFile = '';
            BusVisionDetections.Alignment = -1;
            BusVisionDetections.PreserveElementDimensions = false;
            saveVarsTmp{1} = Simulink.BusElement;
            saveVarsTmp{1}.Name = 'Time';
            saveVarsTmp{1}.Complexity = 'real';
            saveVarsTmp{1}.Dimensions = [1 1];
            saveVarsTmp{1}.DataType = 'double';
            saveVarsTmp{1}.Min = [];
            saveVarsTmp{1}.Max = [];
            saveVarsTmp{1}.DimensionsMode = 'Fixed';
            saveVarsTmp{1}.SamplingMode = 'Sample based';
            saveVarsTmp{1}.DocUnits = '';
            saveVarsTmp{1}.Description = '';
            saveVarsTmp{1}(2, 1) = Simulink.BusElement;
            saveVarsTmp{1}(2, 1).Name = 'Measurement';
            saveVarsTmp{1}(2, 1).Complexity = 'real';
            saveVarsTmp{1}(2, 1).Dimensions = [6 1];
            saveVarsTmp{1}(2, 1).DataType = 'double';
            saveVarsTmp{1}(2, 1).Min = [];
            saveVarsTmp{1}(2, 1).Max = [];
            saveVarsTmp{1}(2, 1).DimensionsMode = 'Fixed';
            saveVarsTmp{1}(2, 1).SamplingMode = 'Sample based';
            saveVarsTmp{1}(2, 1).DocUnits = '';
            saveVarsTmp{1}(2, 1).Description = '';
            saveVarsTmp{1}(3, 1) = Simulink.BusElement;
            saveVarsTmp{1}(3, 1).Name = 'MeasurementNoise';
            saveVarsTmp{1}(3, 1).Complexity = 'real';
            saveVarsTmp{1}(3, 1).Dimensions = [6 6];
            saveVarsTmp{1}(3, 1).DataType = 'double';
            saveVarsTmp{1}(3, 1).Min = [];
            saveVarsTmp{1}(3, 1).Max = [];
            saveVarsTmp{1}(3, 1).DimensionsMode = 'Fixed';
            saveVarsTmp{1}(3, 1).SamplingMode = 'Sample based';
            saveVarsTmp{1}(3, 1).DocUnits = '';
            saveVarsTmp{1}(3, 1).Description = '';
            saveVarsTmp{1}(4, 1) = Simulink.BusElement;
            saveVarsTmp{1}(4, 1).Name = 'SensorIndex';
            saveVarsTmp{1}(4, 1).Complexity = 'real';
            saveVarsTmp{1}(4, 1).Dimensions = [1 1];
            saveVarsTmp{1}(4, 1).DataType = 'double';
            saveVarsTmp{1}(4, 1).Min = [];
            saveVarsTmp{1}(4, 1).Max = [];
            saveVarsTmp{1}(4, 1).DimensionsMode = 'Fixed';
            saveVarsTmp{1}(4, 1).SamplingMode = 'Sample based';
            saveVarsTmp{1}(4, 1).DocUnits = '';
            saveVarsTmp{1}(4, 1).Description = '';
            saveVarsTmp{1}(5, 1) = Simulink.BusElement;
            saveVarsTmp{1}(5, 1).Name = 'ObjectClassID';
            saveVarsTmp{1}(5, 1).Complexity = 'real';
            saveVarsTmp{1}(5, 1).Dimensions = [1 1];
            saveVarsTmp{1}(5, 1).DataType = 'double';
            saveVarsTmp{1}(5, 1).Min = [];
            saveVarsTmp{1}(5, 1).Max = [];
            saveVarsTmp{1}(5, 1).DimensionsMode = 'Fixed';
            saveVarsTmp{1}(5, 1).SamplingMode = 'Sample based';
            saveVarsTmp{1}(5, 1).DocUnits = '';
            saveVarsTmp{1}(5, 1).Description = '';
            saveVarsTmp{1}(6, 1) = Simulink.BusElement;
            saveVarsTmp{1}(6, 1).Name = 'MeasurementParameters';
            saveVarsTmp{1}(6, 1).Complexity = 'real';
            saveVarsTmp{1}(6, 1).Dimensions = [1 1];
            saveVarsTmp{1}(6, 1).DataType = ['Bus: BusVisionDetectionsMeasurementPa' ...
                                             'rameters'];
            saveVarsTmp{1}(6, 1).Min = [];
            saveVarsTmp{1}(6, 1).Max = [];
            saveVarsTmp{1}(6, 1).DimensionsMode = 'Fixed';
            saveVarsTmp{1}(6, 1).SamplingMode = 'Sample based';
            saveVarsTmp{1}(6, 1).DocUnits = '';
            saveVarsTmp{1}(6, 1).Description = '';
            saveVarsTmp{1}(7, 1) = Simulink.BusElement;
            saveVarsTmp{1}(7, 1).Name = 'ObjectAttributes';
            saveVarsTmp{1}(7, 1).Complexity = 'real';
            saveVarsTmp{1}(7, 1).Dimensions = [1 1];
            saveVarsTmp{1}(7, 1).DataType = ['Bus: BusVisionDetectionsObjectAttribu' ...
                                             'tes'];
            saveVarsTmp{1}(7, 1).Min = [];
            saveVarsTmp{1}(7, 1).Max = [];
            saveVarsTmp{1}(7, 1).DimensionsMode = 'Fixed';
            saveVarsTmp{1}(7, 1).SamplingMode = 'Sample based';
            saveVarsTmp{1}(7, 1).DocUnits = '';
            saveVarsTmp{1}(7, 1).Description = '';
            BusVisionDetections.Elements = saveVarsTmp{1};
            clear saveVarsTmp;

            % Create BusVisionDetectionsMeasurementParameters
            BusVisionDetectionsMeasurementParameters = Simulink.Bus;
            BusVisionDetectionsMeasurementParameters.Description = '';
            BusVisionDetectionsMeasurementParameters.DataScope = 'Auto';
            BusVisionDetectionsMeasurementParameters.HeaderFile = '';
            BusVisionDetectionsMeasurementParameters.Alignment = -1;
            BusVisionDetectionsMeasurementParameters.PreserveElementDimensions = false;
            saveVarsTmp{1} = Simulink.BusElement;
            saveVarsTmp{1}.Name = 'Frame';
            saveVarsTmp{1}.Complexity = 'real';
            saveVarsTmp{1}.Dimensions = [1 1];
            saveVarsTmp{1}.DataType = 'Enum: drivingCoordinateFrameType';
            saveVarsTmp{1}.Min = [];
            saveVarsTmp{1}.Max = [];
            saveVarsTmp{1}.DimensionsMode = 'Fixed';
            saveVarsTmp{1}.SamplingMode = 'Sample based';
            saveVarsTmp{1}.DocUnits = '';
            saveVarsTmp{1}.Description = '';
            saveVarsTmp{1}(2, 1) = Simulink.BusElement;
            saveVarsTmp{1}(2, 1).Name = 'OriginPosition';
            saveVarsTmp{1}(2, 1).Complexity = 'real';
            saveVarsTmp{1}(2, 1).Dimensions = [3 1];
            saveVarsTmp{1}(2, 1).DataType = 'double';
            saveVarsTmp{1}(2, 1).Min = [];
            saveVarsTmp{1}(2, 1).Max = [];
            saveVarsTmp{1}(2, 1).DimensionsMode = 'Fixed';
            saveVarsTmp{1}(2, 1).SamplingMode = 'Sample based';
            saveVarsTmp{1}(2, 1).DocUnits = '';
            saveVarsTmp{1}(2, 1).Description = '';
            saveVarsTmp{1}(3, 1) = Simulink.BusElement;
            saveVarsTmp{1}(3, 1).Name = 'Orientation';
            saveVarsTmp{1}(3, 1).Complexity = 'real';
            saveVarsTmp{1}(3, 1).Dimensions = [3 3];
            saveVarsTmp{1}(3, 1).DataType = 'double';
            saveVarsTmp{1}(3, 1).Min = [];
            saveVarsTmp{1}(3, 1).Max = [];
            saveVarsTmp{1}(3, 1).DimensionsMode = 'Fixed';
            saveVarsTmp{1}(3, 1).SamplingMode = 'Sample based';
            saveVarsTmp{1}(3, 1).DocUnits = '';
            saveVarsTmp{1}(3, 1).Description = '';
            saveVarsTmp{1}(4, 1) = Simulink.BusElement;
            saveVarsTmp{1}(4, 1).Name = 'HasVelocity';
            saveVarsTmp{1}(4, 1).Complexity = 'real';
            saveVarsTmp{1}(4, 1).Dimensions = [1 1];
            saveVarsTmp{1}(4, 1).DataType = 'boolean';
            saveVarsTmp{1}(4, 1).Min = [];
            saveVarsTmp{1}(4, 1).Max = [];
            saveVarsTmp{1}(4, 1).DimensionsMode = 'Fixed';
            saveVarsTmp{1}(4, 1).SamplingMode = 'Sample based';
            saveVarsTmp{1}(4, 1).DocUnits = '';
            saveVarsTmp{1}(4, 1).Description = '';
            BusVisionDetectionsMeasurementParameters.Elements = saveVarsTmp{1};
            clear saveVarsTmp;

            % Create BusVisionDetectionsObjectAttributes
            BusVisionDetectionsObjectAttributes = Simulink.Bus;
            BusVisionDetectionsObjectAttributes.Description = '';
            BusVisionDetectionsObjectAttributes.DataScope = 'Auto';
            BusVisionDetectionsObjectAttributes.HeaderFile = '';
            BusVisionDetectionsObjectAttributes.Alignment = -1;
            BusVisionDetectionsObjectAttributes.PreserveElementDimensions = false;
            saveVarsTmp{1} = Simulink.BusElement;
            saveVarsTmp{1}.Name = 'TargetIndex';
            saveVarsTmp{1}.Complexity = 'real';
            saveVarsTmp{1}.Dimensions = [1 1];
            saveVarsTmp{1}.DataType = 'double';
            saveVarsTmp{1}.Min = [];
            saveVarsTmp{1}.Max = [];
            saveVarsTmp{1}.DimensionsMode = 'Fixed';
            saveVarsTmp{1}.SamplingMode = 'Sample based';
            saveVarsTmp{1}.DocUnits = '';
            saveVarsTmp{1}.Description = '';
            BusVisionDetectionsObjectAttributes.Elements = saveVarsTmp{1};
            clear saveVarsTmp;

            % Create BusMultiObjectTracker1Tracks
            BusMultiObjectTracker1 = Simulink.Bus;
            BusMultiObjectTracker1.Description = '';
            BusMultiObjectTracker1.DataScope = 'Auto';
            BusMultiObjectTracker1.HeaderFile = '';
            BusMultiObjectTracker1.Alignment = -1;
            BusMultiObjectTracker1.PreserveElementDimensions = false;
            saveVarsTmp{1} = Simulink.BusElement;
            saveVarsTmp{1}.Name = 'NumTracks';
            saveVarsTmp{1}.Complexity = 'real';
            saveVarsTmp{1}.Dimensions = [1 1];
            saveVarsTmp{1}.DataType = 'double';
            saveVarsTmp{1}.Min = [];
            saveVarsTmp{1}.Max = [];
            saveVarsTmp{1}.DimensionsMode = 'Fixed';
            saveVarsTmp{1}.SamplingMode = 'Sample based';
            saveVarsTmp{1}.DocUnits = '';
            saveVarsTmp{1}.Description = '';
            saveVarsTmp{1}(2, 1) = Simulink.BusElement;
            saveVarsTmp{1}(2, 1).Name = 'Tracks';
            saveVarsTmp{1}(2, 1).Complexity = 'real';
            saveVarsTmp{1}(2, 1).Dimensions = [20 1];
            saveVarsTmp{1}(2, 1).DataType = 'Bus: BusMultiObjectTracker1Tracks';
            saveVarsTmp{1}(2, 1).Min = [];
            saveVarsTmp{1}(2, 1).Max = [];
            saveVarsTmp{1}(2, 1).DimensionsMode = 'Fixed';
            saveVarsTmp{1}(2, 1).SamplingMode = 'Sample based';
            saveVarsTmp{1}(2, 1).DocUnits = '';
            saveVarsTmp{1}(2, 1).Description = '';
            BusMultiObjectTracker1.Elements = saveVarsTmp{1};
            clear saveVarsTmp;

            % Create BusMultiObjectTracker1Tracks
            BusMultiObjectTracker1Tracks = Simulink.Bus;
            BusMultiObjectTracker1Tracks.Description = '';
            BusMultiObjectTracker1Tracks.DataScope = 'Auto';
            BusMultiObjectTracker1Tracks.HeaderFile = '';
            BusMultiObjectTracker1Tracks.Alignment = -1;
            BusMultiObjectTracker1Tracks.PreserveElementDimensions = false;
            saveVarsTmp{1} = Simulink.BusElement;
            saveVarsTmp{1}.Name = 'TrackID';
            saveVarsTmp{1}.Complexity = 'real';
            saveVarsTmp{1}.Dimensions = [1 1];
            saveVarsTmp{1}.DataType = 'uint32';
            saveVarsTmp{1}.Min = [];
            saveVarsTmp{1}.Max = [];
            saveVarsTmp{1}.DimensionsMode = 'Fixed';
            saveVarsTmp{1}.SamplingMode = 'Sample based';
            saveVarsTmp{1}.DocUnits = '';
            saveVarsTmp{1}.Description = '';
            saveVarsTmp{1}(2, 1) = Simulink.BusElement;
            saveVarsTmp{1}(2, 1).Name = 'BranchID';
            saveVarsTmp{1}(2, 1).Complexity = 'real';
            saveVarsTmp{1}(2, 1).Dimensions = [1 1];
            saveVarsTmp{1}(2, 1).DataType = 'uint32';
            saveVarsTmp{1}(2, 1).Min = [];
            saveVarsTmp{1}(2, 1).Max = [];
            saveVarsTmp{1}(2, 1).DimensionsMode = 'Fixed';
            saveVarsTmp{1}(2, 1).SamplingMode = 'Sample based';
            saveVarsTmp{1}(2, 1).DocUnits = '';
            saveVarsTmp{1}(2, 1).Description = '';
            saveVarsTmp{1}(3, 1) = Simulink.BusElement;
            saveVarsTmp{1}(3, 1).Name = 'SourceIndex';
            saveVarsTmp{1}(3, 1).Complexity = 'real';
            saveVarsTmp{1}(3, 1).Dimensions = [1 1];
            saveVarsTmp{1}(3, 1).DataType = 'uint32';
            saveVarsTmp{1}(3, 1).Min = [];
            saveVarsTmp{1}(3, 1).Max = [];
            saveVarsTmp{1}(3, 1).DimensionsMode = 'Fixed';
            saveVarsTmp{1}(3, 1).SamplingMode = 'Sample based';
            saveVarsTmp{1}(3, 1).DocUnits = '';
            saveVarsTmp{1}(3, 1).Description = '';
            saveVarsTmp{1}(4, 1) = Simulink.BusElement;
            saveVarsTmp{1}(4, 1).Name = 'UpdateTime';
            saveVarsTmp{1}(4, 1).Complexity = 'real';
            saveVarsTmp{1}(4, 1).Dimensions = [1 1];
            saveVarsTmp{1}(4, 1).DataType = 'double';
            saveVarsTmp{1}(4, 1).Min = [];
            saveVarsTmp{1}(4, 1).Max = [];
            saveVarsTmp{1}(4, 1).DimensionsMode = 'Fixed';
            saveVarsTmp{1}(4, 1).SamplingMode = 'Sample based';
            saveVarsTmp{1}(4, 1).DocUnits = '';
            saveVarsTmp{1}(4, 1).Description = '';
            saveVarsTmp{1}(5, 1) = Simulink.BusElement;
            saveVarsTmp{1}(5, 1).Name = 'Age';
            saveVarsTmp{1}(5, 1).Complexity = 'real';
            saveVarsTmp{1}(5, 1).Dimensions = [1 1];
            saveVarsTmp{1}(5, 1).DataType = 'uint32';
            saveVarsTmp{1}(5, 1).Min = [];
            saveVarsTmp{1}(5, 1).Max = [];
            saveVarsTmp{1}(5, 1).DimensionsMode = 'Fixed';
            saveVarsTmp{1}(5, 1).SamplingMode = 'Sample based';
            saveVarsTmp{1}(5, 1).DocUnits = '';
            saveVarsTmp{1}(5, 1).Description = '';
            saveVarsTmp{1}(6, 1) = Simulink.BusElement;
            saveVarsTmp{1}(6, 1).Name = 'State';
            saveVarsTmp{1}(6, 1).Complexity = 'real';
            saveVarsTmp{1}(6, 1).Dimensions = [6 1];
            saveVarsTmp{1}(6, 1).DataType = 'double';
            saveVarsTmp{1}(6, 1).Min = [];
            saveVarsTmp{1}(6, 1).Max = [];
            saveVarsTmp{1}(6, 1).DimensionsMode = 'Fixed';
            saveVarsTmp{1}(6, 1).SamplingMode = 'Sample based';
            saveVarsTmp{1}(6, 1).DocUnits = '';
            saveVarsTmp{1}(6, 1).Description = '';
            saveVarsTmp{1}(7, 1) = Simulink.BusElement;
            saveVarsTmp{1}(7, 1).Name = 'StateCovariance';
            saveVarsTmp{1}(7, 1).Complexity = 'real';
            saveVarsTmp{1}(7, 1).Dimensions = [6 6];
            saveVarsTmp{1}(7, 1).DataType = 'double';
            saveVarsTmp{1}(7, 1).Min = [];
            saveVarsTmp{1}(7, 1).Max = [];
            saveVarsTmp{1}(7, 1).DimensionsMode = 'Fixed';
            saveVarsTmp{1}(7, 1).SamplingMode = 'Sample based';
            saveVarsTmp{1}(7, 1).DocUnits = '';
            saveVarsTmp{1}(7, 1).Description = '';
            saveVarsTmp{1}(8, 1) = Simulink.BusElement;
            saveVarsTmp{1}(8, 1).Name = 'ObjectClassID';
            saveVarsTmp{1}(8, 1).Complexity = 'real';
            saveVarsTmp{1}(8, 1).Dimensions = [1 1];
            saveVarsTmp{1}(8, 1).DataType = 'double';
            saveVarsTmp{1}(8, 1).Min = [];
            saveVarsTmp{1}(8, 1).Max = [];
            saveVarsTmp{1}(8, 1).DimensionsMode = 'Fixed';
            saveVarsTmp{1}(8, 1).SamplingMode = 'Sample based';
            saveVarsTmp{1}(8, 1).DocUnits = '';
            saveVarsTmp{1}(8, 1).Description = '';
            saveVarsTmp{1}(9, 1) = Simulink.BusElement;
            saveVarsTmp{1}(9, 1).Name = 'TrackLogic';
            saveVarsTmp{1}(9, 1).Complexity = 'real';
            saveVarsTmp{1}(9, 1).Dimensions = [1 1];
            saveVarsTmp{1}(9, 1).DataType = 'Enum: trackLogicType';
            saveVarsTmp{1}(9, 1).Min = [];
            saveVarsTmp{1}(9, 1).Max = [];
            saveVarsTmp{1}(9, 1).DimensionsMode = 'Fixed';
            saveVarsTmp{1}(9, 1).SamplingMode = 'Sample based';
            saveVarsTmp{1}(9, 1).DocUnits = '';
            saveVarsTmp{1}(9, 1).Description = '';
            saveVarsTmp{1}(10, 1) = Simulink.BusElement;
            saveVarsTmp{1}(10, 1).Name = 'TrackLogicState';
            saveVarsTmp{1}(10, 1).Complexity = 'real';
            saveVarsTmp{1}(10, 1).Dimensions = [1 3];
            saveVarsTmp{1}(10, 1).DataType = 'boolean';
            saveVarsTmp{1}(10, 1).Min = [];
            saveVarsTmp{1}(10, 1).Max = [];
            saveVarsTmp{1}(10, 1).DimensionsMode = 'Fixed';
            saveVarsTmp{1}(10, 1).SamplingMode = 'Sample based';
            saveVarsTmp{1}(10, 1).DocUnits = '';
            saveVarsTmp{1}(10, 1).Description = '';
            saveVarsTmp{1}(11, 1) = Simulink.BusElement;
            saveVarsTmp{1}(11, 1).Name = 'IsConfirmed';
            saveVarsTmp{1}(11, 1).Complexity = 'real';
            saveVarsTmp{1}(11, 1).Dimensions = [1 1];
            saveVarsTmp{1}(11, 1).DataType = 'boolean';
            saveVarsTmp{1}(11, 1).Min = [];
            saveVarsTmp{1}(11, 1).Max = [];
            saveVarsTmp{1}(11, 1).DimensionsMode = 'Fixed';
            saveVarsTmp{1}(11, 1).SamplingMode = 'Sample based';
            saveVarsTmp{1}(11, 1).DocUnits = '';
            saveVarsTmp{1}(11, 1).Description = '';
            saveVarsTmp{1}(12, 1) = Simulink.BusElement;
            saveVarsTmp{1}(12, 1).Name = 'IsCoasted';
            saveVarsTmp{1}(12, 1).Complexity = 'real';
            saveVarsTmp{1}(12, 1).Dimensions = [1 1];
            saveVarsTmp{1}(12, 1).DataType = 'boolean';
            saveVarsTmp{1}(12, 1).Min = [];
            saveVarsTmp{1}(12, 1).Max = [];
            saveVarsTmp{1}(12, 1).DimensionsMode = 'Fixed';
            saveVarsTmp{1}(12, 1).SamplingMode = 'Sample based';
            saveVarsTmp{1}(12, 1).DocUnits = '';
            saveVarsTmp{1}(12, 1).Description = '';
            saveVarsTmp{1}(13, 1) = Simulink.BusElement;
            saveVarsTmp{1}(13, 1).Name = 'IsSelfReported';
            saveVarsTmp{1}(13, 1).Complexity = 'real';
            saveVarsTmp{1}(13, 1).Dimensions = [1 1];
            saveVarsTmp{1}(13, 1).DataType = 'boolean';
            saveVarsTmp{1}(13, 1).Min = [];
            saveVarsTmp{1}(13, 1).Max = [];
            saveVarsTmp{1}(13, 1).DimensionsMode = 'Fixed';
            saveVarsTmp{1}(13, 1).SamplingMode = 'Sample based';
            saveVarsTmp{1}(13, 1).DocUnits = '';
            saveVarsTmp{1}(13, 1).Description = '';
            saveVarsTmp{1}(14, 1) = Simulink.BusElement;
            saveVarsTmp{1}(14, 1).Name = 'ObjectAttributes';
            saveVarsTmp{1}(14, 1).Complexity = 'real';
            saveVarsTmp{1}(14, 1).Dimensions = [2 1];
            saveVarsTmp{1}(14, 1).DataType = ['Bus: BusRadarDetectionsObjectAttribu' ...
                                              'tes'];
            saveVarsTmp{1}(14, 1).Min = [];
            saveVarsTmp{1}(14, 1).Max = [];
            saveVarsTmp{1}(14, 1).DimensionsMode = 'Fixed';
            saveVarsTmp{1}(14, 1).SamplingMode = 'Sample based';
            saveVarsTmp{1}(14, 1).DocUnits = '';
            saveVarsTmp{1}(14, 1).Description = '';
            BusMultiObjectTracker1Tracks.Elements = saveVarsTmp{1};
            clear saveVarsTmp;

            % Bus object: BusPathPointData
            clear elems;
            elems(1) = Simulink.BusElement;
            elems(1).Name = 'NumPathPoints';
            elems(1).Dimensions = [1 1];
            elems(1).DimensionsMode = 'Fixed';
            elems(1).DataType = 'double';
            elems(1).Complexity = 'real';
            elems(1).Min = [];
            elems(1).Max = [];
            elems(1).DocUnits = '';
            elems(1).Description = '';

            elems(2) = Simulink.BusElement;
            elems(2).Name = 'Path';
            elems(2).Dimensions = [obj.RefPathSize 3];
            elems(2).DimensionsMode = 'Fixed';
            elems(2).DataType = 'double';
            elems(2).Complexity = 'real';
            elems(2).Min = [];
            elems(2).Max = [];
            elems(2).DocUnits = '';
            elems(2).Description = '';

            elems(3) = Simulink.BusElement;
            elems(3).Name = 'Speed';
            elems(3).Dimensions = [obj.RefPathSize 1];
            elems(3).DimensionsMode = 'Fixed';
            elems(3).DataType = 'double';
            elems(3).Complexity = 'real';
            elems(3).Min = [];
            elems(3).Max = [];
            elems(3).DocUnits = '';
            elems(3).Description = '';

            elems(4) = Simulink.BusElement;
            elems(4).Name = 'Time';
            elems(4).Dimensions = [obj.RefPathSize 1];
            elems(4).DimensionsMode = 'Fixed';
            elems(4).DataType = 'double';
            elems(4).Complexity = 'real';
            elems(4).Min = [];
            elems(4).Max = [];
            elems(4).DocUnits = '';
            elems(4).Description = '';

            elems(5) = Simulink.BusElement;
            elems(5).Name = 'WaitTime';
            elems(5).Dimensions = [obj.RefPathSize 1];
            elems(5).DimensionsMode = 'Fixed';
            elems(5).DataType = 'double';
            elems(5).Complexity = 'real';
            elems(5).Min = [];
            elems(5).Max = [];
            elems(5).DocUnits = '';
            elems(5).Description = '';

            BusPathPointData = Simulink.Bus;
            BusPathPointData.HeaderFile = '';
            BusPathPointData.Description = '';
            BusPathPointData.DataScope = 'Auto';
            BusPathPointData.Alignment = -1;
            BusPathPointData.PreserveElementDimensions = 0;
            BusPathPointData.Elements = elems;
            clear elems;

            % Bus object: BusRefPath1
            clear elems;
            elems(1) = Simulink.BusElement;
            elems(1).Name = 's';
            elems(1).Dimensions = [obj.RefPathSize 1];
            elems(1).DimensionsMode = 'Variable';
            elems(1).DataType = 'double';
            elems(1).Complexity = 'real';
            elems(1).Min = [];
            elems(1).Max = [];
            elems(1).DocUnits = '';
            elems(1).Description = '';

            elems(2) = Simulink.BusElement;
            elems(2).Name = 'x';
            elems(2).Dimensions = [obj.RefPathSize 1];
            elems(2).DimensionsMode = 'Variable';
            elems(2).DataType = 'double';
            elems(2).Complexity = 'real';
            elems(2).Min = [];
            elems(2).Max = [];
            elems(2).DocUnits = '';
            elems(2).Description = '';

            elems(3) = Simulink.BusElement;
            elems(3).Name = 'y';
            elems(3).Dimensions = [obj.RefPathSize 1];
            elems(3).DimensionsMode = 'Variable';
            elems(3).DataType = 'double';
            elems(3).Complexity = 'real';
            elems(3).Min = [];
            elems(3).Max = [];
            elems(3).DocUnits = '';
            elems(3).Description = '';

            elems(4) = Simulink.BusElement;
            elems(4).Name = 'theta';
            elems(4).Dimensions = [obj.RefPathSize 1];
            elems(4).DimensionsMode = 'Variable';
            elems(4).DataType = 'double';
            elems(4).Complexity = 'real';
            elems(4).Min = [];
            elems(4).Max = [];
            elems(4).DocUnits = '';
            elems(4).Description = '';

            elems(5) = Simulink.BusElement;
            elems(5).Name = 'kappa';
            elems(5).Dimensions = [obj.RefPathSize 1];
            elems(5).DimensionsMode = 'Variable';
            elems(5).DataType = 'double';
            elems(5).Complexity = 'real';
            elems(5).Min = [];
            elems(5).Max = [];
            elems(5).DocUnits = '';
            elems(5).Description = '';

            elems(6) = Simulink.BusElement;
            elems(6).Name = 'elev';
            elems(6).Dimensions = [obj.RefPathSize 1];
            elems(6).DimensionsMode = 'Variable';
            elems(6).DataType = 'double';
            elems(6).Complexity = 'real';
            elems(6).Min = [];
            elems(6).Max = [];
            elems(6).DocUnits = '';
            elems(6).Description = '';

            elems(7) = Simulink.BusElement;
            elems(7).Name = 'grade';
            elems(7).Dimensions = [obj.RefPathSize 1];
            elems(7).DimensionsMode = 'Variable';
            elems(7).DataType = 'double';
            elems(7).Complexity = 'real';
            elems(7).Min = [];
            elems(7).Max = [];
            elems(7).DocUnits = '';
            elems(7).Description = '';

            elems(8) = Simulink.BusElement;
            elems(8).Name = 'bank';
            elems(8).Dimensions = [obj.RefPathSize 1];
            elems(8).DimensionsMode = 'Variable';
            elems(8).DataType = 'double';
            elems(8).Complexity = 'real';
            elems(8).Min = [];
            elems(8).Max = [];
            elems(8).DocUnits = '';
            elems(8).Description = '';

            elems(9) = Simulink.BusElement;
            elems(9).Name = 'speed';
            elems(9).Dimensions = [obj.RefPathSize 1];
            elems(9).DimensionsMode = 'Variable';
            elems(9).DataType = 'double';
            elems(9).Complexity = 'real';
            elems(9).Min = [];
            elems(9).Max = [];
            elems(9).DocUnits = '';
            elems(9).Description = '';

            elems(10) = Simulink.BusElement;
            elems(10).Name = 'time';
            elems(10).Dimensions = [obj.RefPathSize 1];
            elems(10).DimensionsMode = 'Variable';
            elems(10).DataType = 'double';
            elems(10).Complexity = 'real';
            elems(10).Min = [];
            elems(10).Max = [];
            elems(10).DocUnits = '';
            elems(10).Description = '';

            elems(11) = Simulink.BusElement;
            elems(11).Name = 'waitTime';
            elems(11).Dimensions = [obj.RefPathSize 1];
            elems(11).DimensionsMode = 'Variable';
            elems(11).DataType = 'double';
            elems(11).Complexity = 'real';
            elems(11).Min = [];
            elems(11).Max = [];
            elems(11).DocUnits = '';
            elems(11).Description = '';


            BusRefPath = Simulink.Bus;
            BusRefPath.HeaderFile = '';
            BusRefPath.Description = '';
            BusRefPath.DataScope = 'Auto';
            BusRefPath.Alignment = -1;
            BusRefPath.PreserveElementDimensions = 0;
            BusRefPath.Elements = elems;
            clear elems;

            % Bus object: BusPathPointTiming
            clear elems;
            elems(1) = Simulink.BusElement;
            elems(1).Name = 'Time';
            elems(1).Dimensions = [1 1];
            elems(1).DimensionsMode = 'Fixed';
            elems(1).DataType = 'double';
            elems(1).Complexity = 'real';
            elems(1).Min = [];
            elems(1).Max = [];
            elems(1).DocUnits = '';
            elems(1).Description = '';

            elems(2) = Simulink.BusElement;
            elems(2).Name = 'Speed';
            elems(2).Dimensions = [1 1];
            elems(2).DimensionsMode = 'Fixed';
            elems(2).DataType = 'double';
            elems(2).Complexity = 'real';
            elems(2).Min = [];
            elems(2).Max = [];
            elems(2).DocUnits = '';
            elems(2).Description = '';

            elems(3) = Simulink.BusElement;
            elems(3).Name = 'WaitTime';
            elems(3).Dimensions = [1 1];
            elems(3).DimensionsMode = 'Fixed';
            elems(3).DataType = 'double';
            elems(3).Complexity = 'real';
            elems(3).Min = [];
            elems(3).Max = [];
            elems(3).DocUnits = '';
            elems(3).Description = '';

            BusPathPointTiming = Simulink.Bus;
            BusPathPointTiming.HeaderFile = '';
            BusPathPointTiming.Description = '';
            BusPathPointTiming.DataScope = 'Auto';
            BusPathPointTiming.Alignment = -1;
            BusPathPointTiming.PreserveElementDimensions = 0;
            BusPathPointTiming.Elements = elems;
            clear elems;
            assignin('base','BusPathPointTiming', BusPathPointTiming);

            % Bus object: BusPathTarget
            clear elems;
            elems(1) = Simulink.BusElement;
            elems(1).Name = 'Path';
            elems(1).Dimensions = [obj.RefPathSize 3];
            elems(1).DimensionsMode = 'Fixed';
            elems(1).DataType = 'double';
            elems(1).Complexity = 'real';
            elems(1).Min = [];
            elems(1).Max = [];
            elems(1).DocUnits = '';
            elems(1).Description = '';

            elems(2) = Simulink.BusElement;
            elems(2).Name = 'NumPoints';
            elems(2).Dimensions = [1 1];
            elems(2).DimensionsMode = 'Fixed';
            elems(2).DataType = 'uint64';
            elems(2).Complexity = 'real';
            elems(2).Min = [];
            elems(2).Max = [];
            elems(2).DocUnits = '';
            elems(2).Description = '';

            elems(3) = Simulink.BusElement;
            elems(3).Name = 'HasTimings';
            elems(3).Dimensions = [1 1];
            elems(3).DimensionsMode = 'Fixed';
            elems(3).DataType = 'boolean';
            elems(3).Complexity = 'real';
            elems(3).Min = [];
            elems(3).Max = [];
            elems(3).DocUnits = '';
            elems(3).Description = '';

            elems(4) = Simulink.BusElement;
            elems(4).Name = 'Timings';
            elems(4).Dimensions = [1 obj.RefPathSize];
            elems(4).DimensionsMode = 'Fixed';
            elems(4).DataType = 'Bus: BusPathPointTiming';
            elems(4).Complexity = 'real';
            elems(4).Min = [];
            elems(4).Max = [];
            elems(4).DocUnits = '';
            elems(4).Description = '';

            BusPathTarget = Simulink.Bus;
            BusPathTarget.HeaderFile = '';
            BusPathTarget.Description = '';
            BusPathTarget.DataScope = 'Auto';
            BusPathTarget.Alignment = -1;
            BusPathTarget.PreserveElementDimensions = 0;
            BusPathTarget.Elements = elems;
            clear elems;

            % Bus object: BusDeviations
            clear elems;
            elems(1) = Simulink.BusElement;
            elems(1).Name = 'CurvatureDeviation';
            elems(1).Dimensions = [obj.PredictionHorizon 1];
            elems(1).DimensionsMode = 'Fixed';
            elems(1).DataType = 'double';
            elems(1).Complexity = 'real';
            elems(1).Min = [];
            elems(1).Max = [];
            elems(1).DocUnits = '';
            elems(1).Description = '';

            elems(2) = Simulink.BusElement;
            elems(2).Name = 'LateralDeviation';
            elems(2).Dimensions = 1;
            elems(2).DimensionsMode = 'Fixed';
            elems(2).DataType = 'double';
            elems(2).Complexity = 'real';
            elems(2).Min = [];
            elems(2).Max = [];
            elems(2).DocUnits = '';
            elems(2).Description = '';

            elems(3) = Simulink.BusElement;
            elems(3).Name = 'RelativeYawAngle';
            elems(3).Dimensions = 1;
            elems(3).DimensionsMode = 'Fixed';
            elems(3).DataType = 'double';
            elems(3).Complexity = 'real';
            elems(3).Min = [];
            elems(3).Max = [];
            elems(3).DocUnits = '';
            elems(3).Description = '';

            BusDeviations = Simulink.Bus;
            BusDeviations.HeaderFile = '';
            BusDeviations.Description = '';
            BusDeviations.DataScope = 'Auto';
            BusDeviations.Alignment = -1;
            BusDeviations.PreserveElementDimensions = 0;
            BusDeviations.Elements = elems;
            clear elems;

            % Create slBus1_LaneBoundaries
            slBus1_LaneBoundaries = Simulink.Bus;
            slBus1_LaneBoundaries.Description = '';
            slBus1_LaneBoundaries.DataScope = 'Auto';
            slBus1_LaneBoundaries.HeaderFile = '';
            slBus1_LaneBoundaries.Alignment = -1;
            slBus1_LaneBoundaries.PreserveElementDimensions = false;
            saveVarsTmp{1} = Simulink.BusElement;
            saveVarsTmp{1}.Name = 'Curvature';
            saveVarsTmp{1}.Complexity = 'real';
            saveVarsTmp{1}.Dimensions = [1 1];
            saveVarsTmp{1}.DataType = 'double';
            saveVarsTmp{1}.Min = [];
            saveVarsTmp{1}.Max = [];
            saveVarsTmp{1}.DimensionsMode = 'Fixed';
            saveVarsTmp{1}.SamplingMode = 'Sample based';
            saveVarsTmp{1}.DocUnits = '';
            saveVarsTmp{1}.Description = '';
            saveVarsTmp{1}(2, 1) = Simulink.BusElement;
            saveVarsTmp{1}(2, 1).Name = 'CurvatureDerivative';
            saveVarsTmp{1}(2, 1).Complexity = 'real';
            saveVarsTmp{1}(2, 1).Dimensions = [1 1];
            saveVarsTmp{1}(2, 1).DataType = 'double';
            saveVarsTmp{1}(2, 1).Min = [];
            saveVarsTmp{1}(2, 1).Max = [];
            saveVarsTmp{1}(2, 1).DimensionsMode = 'Fixed';
            saveVarsTmp{1}(2, 1).SamplingMode = 'Sample based';
            saveVarsTmp{1}(2, 1).DocUnits = '';
            saveVarsTmp{1}(2, 1).Description = '';
            saveVarsTmp{1}(3, 1) = Simulink.BusElement;
            saveVarsTmp{1}(3, 1).Name = 'CurveLength';
            saveVarsTmp{1}(3, 1).Complexity = 'real';
            saveVarsTmp{1}(3, 1).Dimensions = [1 1];
            saveVarsTmp{1}(3, 1).DataType = 'double';
            saveVarsTmp{1}(3, 1).Min = [];
            saveVarsTmp{1}(3, 1).Max = [];
            saveVarsTmp{1}(3, 1).DimensionsMode = 'Fixed';
            saveVarsTmp{1}(3, 1).SamplingMode = 'Sample based';
            saveVarsTmp{1}(3, 1).DocUnits = '';
            saveVarsTmp{1}(3, 1).Description = '';
            saveVarsTmp{1}(4, 1) = Simulink.BusElement;
            saveVarsTmp{1}(4, 1).Name = 'HeadingAngle';
            saveVarsTmp{1}(4, 1).Complexity = 'real';
            saveVarsTmp{1}(4, 1).Dimensions = [1 1];
            saveVarsTmp{1}(4, 1).DataType = 'double';
            saveVarsTmp{1}(4, 1).Min = [];
            saveVarsTmp{1}(4, 1).Max = [];
            saveVarsTmp{1}(4, 1).DimensionsMode = 'Fixed';
            saveVarsTmp{1}(4, 1).SamplingMode = 'Sample based';
            saveVarsTmp{1}(4, 1).DocUnits = '';
            saveVarsTmp{1}(4, 1).Description = '';
            saveVarsTmp{1}(5, 1) = Simulink.BusElement;
            saveVarsTmp{1}(5, 1).Name = 'LateralOffset';
            saveVarsTmp{1}(5, 1).Complexity = 'real';
            saveVarsTmp{1}(5, 1).Dimensions = [1 1];
            saveVarsTmp{1}(5, 1).DataType = 'double';
            saveVarsTmp{1}(5, 1).Min = [];
            saveVarsTmp{1}(5, 1).Max = [];
            saveVarsTmp{1}(5, 1).DimensionsMode = 'Fixed';
            saveVarsTmp{1}(5, 1).SamplingMode = 'Sample based';
            saveVarsTmp{1}(5, 1).DocUnits = '';
            saveVarsTmp{1}(5, 1).Description = '';
            saveVarsTmp{1}(6, 1) = Simulink.BusElement;
            saveVarsTmp{1}(6, 1).Name = 'BoundaryType';
            saveVarsTmp{1}(6, 1).Complexity = 'real';
            saveVarsTmp{1}(6, 1).Dimensions = [1 1];
            saveVarsTmp{1}(6, 1).DataType = 'uint8';
            saveVarsTmp{1}(6, 1).Min = [];
            saveVarsTmp{1}(6, 1).Max = [];
            saveVarsTmp{1}(6, 1).DimensionsMode = 'Fixed';
            saveVarsTmp{1}(6, 1).SamplingMode = 'Sample based';
            saveVarsTmp{1}(6, 1).DocUnits = '';
            saveVarsTmp{1}(6, 1).Description = '';
            saveVarsTmp{1}(7, 1) = Simulink.BusElement;
            saveVarsTmp{1}(7, 1).Name = 'Strength';
            saveVarsTmp{1}(7, 1).Complexity = 'real';
            saveVarsTmp{1}(7, 1).Dimensions = [1 1];
            saveVarsTmp{1}(7, 1).DataType = 'double';
            saveVarsTmp{1}(7, 1).Min = [];
            saveVarsTmp{1}(7, 1).Max = [];
            saveVarsTmp{1}(7, 1).DimensionsMode = 'Fixed';
            saveVarsTmp{1}(7, 1).SamplingMode = 'Sample based';
            saveVarsTmp{1}(7, 1).DocUnits = '';
            saveVarsTmp{1}(7, 1).Description = '';
            saveVarsTmp{1}(8, 1) = Simulink.BusElement;
            saveVarsTmp{1}(8, 1).Name = 'Width';
            saveVarsTmp{1}(8, 1).Complexity = 'real';
            saveVarsTmp{1}(8, 1).Dimensions = [1 1];
            saveVarsTmp{1}(8, 1).DataType = 'double';
            saveVarsTmp{1}(8, 1).Min = [];
            saveVarsTmp{1}(8, 1).Max = [];
            saveVarsTmp{1}(8, 1).DimensionsMode = 'Fixed';
            saveVarsTmp{1}(8, 1).SamplingMode = 'Sample based';
            saveVarsTmp{1}(8, 1).DocUnits = '';
            saveVarsTmp{1}(8, 1).Description = '';
            slBus1_LaneBoundaries.Elements = saveVarsTmp{1};
            clear saveVarsTmp;


            % Create LaneBus
            LaneBus = Simulink.Bus;
            LaneBus.Description = '';
            LaneBus.DataScope = 'Auto';
            LaneBus.HeaderFile = '';
            LaneBus.Alignment = -1;
            LaneBus.PreserveElementDimensions = false;
            saveVarsTmp{1} = Simulink.BusElement;
            saveVarsTmp{1}.Name = 'Time';
            saveVarsTmp{1}.Complexity = 'real';
            saveVarsTmp{1}.Dimensions = [1 1];
            saveVarsTmp{1}.DataType = 'double';
            saveVarsTmp{1}.Min = [];
            saveVarsTmp{1}.Max = [];
            saveVarsTmp{1}.DimensionsMode = 'Fixed';
            saveVarsTmp{1}.SamplingMode = 'Sample based';
            saveVarsTmp{1}.DocUnits = '';
            saveVarsTmp{1}.Description = '';
            saveVarsTmp{1}(2, 1) = Simulink.BusElement;
            saveVarsTmp{1}(2, 1).Name = 'IsValidTime';
            saveVarsTmp{1}(2, 1).Complexity = 'real';
            saveVarsTmp{1}(2, 1).Dimensions = [1 1];
            saveVarsTmp{1}(2, 1).DataType = 'boolean';
            saveVarsTmp{1}(2, 1).Min = [];
            saveVarsTmp{1}(2, 1).Max = [];
            saveVarsTmp{1}(2, 1).DimensionsMode = 'Fixed';
            saveVarsTmp{1}(2, 1).SamplingMode = 'Sample based';
            saveVarsTmp{1}(2, 1).DocUnits = '';
            saveVarsTmp{1}(2, 1).Description = '';
            saveVarsTmp{1}(3, 1) = Simulink.BusElement;
            saveVarsTmp{1}(3, 1).Name = 'SensorIndex';
            saveVarsTmp{1}(3, 1).Complexity = 'real';
            saveVarsTmp{1}(3, 1).Dimensions = [1 1];
            saveVarsTmp{1}(3, 1).DataType = 'double';
            saveVarsTmp{1}(3, 1).Min = [];
            saveVarsTmp{1}(3, 1).Max = [];
            saveVarsTmp{1}(3, 1).DimensionsMode = 'Fixed';
            saveVarsTmp{1}(3, 1).SamplingMode = 'Sample based';
            saveVarsTmp{1}(3, 1).DocUnits = '';
            saveVarsTmp{1}(3, 1).Description = '';
            saveVarsTmp{1}(4, 1) = Simulink.BusElement;
            saveVarsTmp{1}(4, 1).Name = 'NumLaneBoundaries';
            saveVarsTmp{1}(4, 1).Complexity = 'real';
            saveVarsTmp{1}(4, 1).Dimensions = [1 1];
            saveVarsTmp{1}(4, 1).DataType = 'double';
            saveVarsTmp{1}(4, 1).Min = [];
            saveVarsTmp{1}(4, 1).Max = [];
            saveVarsTmp{1}(4, 1).DimensionsMode = 'Fixed';
            saveVarsTmp{1}(4, 1).SamplingMode = 'Sample based';
            saveVarsTmp{1}(4, 1).DocUnits = '';
            saveVarsTmp{1}(4, 1).Description = '';
            saveVarsTmp{1}(5, 1) = Simulink.BusElement;
            saveVarsTmp{1}(5, 1).Name = 'LaneBoundaries';
            saveVarsTmp{1}(5, 1).Complexity = 'real';
            saveVarsTmp{1}(5, 1).Dimensions = [2 1];
            saveVarsTmp{1}(5, 1).DataType = 'slBus1_LaneBoundaries';
            saveVarsTmp{1}(5, 1).Min = [];
            saveVarsTmp{1}(5, 1).Max = [];
            saveVarsTmp{1}(5, 1).DimensionsMode = 'Fixed';
            saveVarsTmp{1}(5, 1).SamplingMode = 'Sample based';
            saveVarsTmp{1}(5, 1).DocUnits = '';
            saveVarsTmp{1}(5, 1).Description = '';
            LaneBus.Elements = saveVarsTmp{1};
            clear saveVarsTmp;

            % Create BusLaneSensor
            BusLaneSensor = Simulink.Bus;
            BusLaneSensor.Description = ['Describes sensor structure interface for lan' ...
                                         'e sensor'];
            BusLaneSensor.DataScope = 'Auto';
            BusLaneSensor.HeaderFile = '';
            BusLaneSensor.Alignment = -1;
            saveVarsTmp{1} = Simulink.BusElement;
            saveVarsTmp{1}.Name = 'Left';
            saveVarsTmp{1}.Complexity = 'real';
            saveVarsTmp{1}.Dimensions = 1;
            saveVarsTmp{1}.DataType = 'Bus: slBus1_LaneBoundaries';
            saveVarsTmp{1}.Min = [];
            saveVarsTmp{1}.Max = [];
            saveVarsTmp{1}.DimensionsMode = 'Fixed';
            saveVarsTmp{1}.SamplingMode = 'Sample based';
            saveVarsTmp{1}.SampleTime = -1;
            saveVarsTmp{1}.DocUnits = '';
            saveVarsTmp{1}.Description = '';
            saveVarsTmp{1}(2, 1) = Simulink.BusElement;
            saveVarsTmp{1}(2, 1).Name = 'Right';
            saveVarsTmp{1}(2, 1).Complexity = 'real';
            saveVarsTmp{1}(2, 1).Dimensions = 1;
            saveVarsTmp{1}(2, 1).DataType = 'Bus: slBus1_LaneBoundaries';
            saveVarsTmp{1}(2, 1).Min = [];
            saveVarsTmp{1}(2, 1).Max = [];
            saveVarsTmp{1}(2, 1).DimensionsMode = 'Fixed';
            saveVarsTmp{1}(2, 1).SamplingMode = 'Sample based';
            saveVarsTmp{1}(2, 1).SampleTime = -1;
            saveVarsTmp{1}(2, 1).DocUnits = '';
            saveVarsTmp{1}(2, 1).Description = '';
            BusLaneSensor.Elements = saveVarsTmp{1};
            clear saveVarsTmp;

            LaneSensor = Simulink.Bus;
            LaneSensor.Description = ['Describes sensor structure interface for lan' ...
                'e sensor'];
            LaneSensor.DataScope = 'Auto';
            LaneSensor.HeaderFile = '';
            LaneSensor.Alignment = -1;
            saveVarsTmp{1} = Simulink.BusElement;
            saveVarsTmp{1}.Name = 'Left';
            saveVarsTmp{1}.Complexity = 'real';
            saveVarsTmp{1}.Dimensions = 1;
            saveVarsTmp{1}.DataType = 'Bus: LaneSensorBoundaries';
            saveVarsTmp{1}.Min = [];
            saveVarsTmp{1}.Max = [];
            saveVarsTmp{1}.DimensionsMode = 'Fixed';
            saveVarsTmp{1}.SamplingMode = 'Sample based';
            saveVarsTmp{1}.SampleTime = -1;
            saveVarsTmp{1}.DocUnits = '';
            saveVarsTmp{1}.Description = '';
            saveVarsTmp{1}(2, 1) = Simulink.BusElement;
            saveVarsTmp{1}(2, 1).Name = 'Right';
            saveVarsTmp{1}(2, 1).Complexity = 'real';
            saveVarsTmp{1}(2, 1).Dimensions = 1;
            saveVarsTmp{1}(2, 1).DataType = 'Bus: LaneSensorBoundaries';
            saveVarsTmp{1}(2, 1).Min = [];
            saveVarsTmp{1}(2, 1).Max = [];
            saveVarsTmp{1}(2, 1).DimensionsMode = 'Fixed';
            saveVarsTmp{1}(2, 1).SamplingMode = 'Sample based';
            saveVarsTmp{1}(2, 1).SampleTime = -1;
            saveVarsTmp{1}(2, 1).DocUnits = '';
            saveVarsTmp{1}(2, 1).Description = '';
            LaneSensor.Elements = saveVarsTmp{1};
            clear saveVarsTmp;

            LaneSensorBoundaries = Simulink.Bus;
            LaneSensorBoundaries.Description = ['Describes sensor structure interfa' ...
                'ce for lane boundaries from lane s' ...
                'ensor'];
            LaneSensorBoundaries.DataScope = 'Auto';
            LaneSensorBoundaries.HeaderFile = '';
            LaneSensorBoundaries.Alignment = -1;
            saveVarsTmp{1} = Simulink.BusElement;
            saveVarsTmp{1}.Name = 'Curvature';
            saveVarsTmp{1}.Complexity = 'real';
            saveVarsTmp{1}.Dimensions = 1;
            saveVarsTmp{1}.DataType = 'single';
            saveVarsTmp{1}.Min = [];
            saveVarsTmp{1}.Max = [];
            saveVarsTmp{1}.DimensionsMode = 'Fixed';
            saveVarsTmp{1}.SamplingMode = 'Sample based';
            saveVarsTmp{1}.SampleTime = -1;
            saveVarsTmp{1}.DocUnits = 'rad/m';
            saveVarsTmp{1}.Description = '';
            saveVarsTmp{1}(2, 1) = Simulink.BusElement;
            saveVarsTmp{1}(2, 1).Name = 'CurvatureDerivative';
            saveVarsTmp{1}(2, 1).Complexity = 'real';
            saveVarsTmp{1}(2, 1).Dimensions = 1;
            saveVarsTmp{1}(2, 1).DataType = 'single';
            saveVarsTmp{1}(2, 1).Min = [];
            saveVarsTmp{1}(2, 1).Max = [];
            saveVarsTmp{1}(2, 1).DimensionsMode = 'Fixed';
            saveVarsTmp{1}(2, 1).SamplingMode = 'Sample based';
            saveVarsTmp{1}(2, 1).SampleTime = -1;
            saveVarsTmp{1}(2, 1).DocUnits = 'rad/m^2';
            saveVarsTmp{1}(2, 1).Description = '';
            saveVarsTmp{1}(3, 1) = Simulink.BusElement;
            saveVarsTmp{1}(3, 1).Name = 'HeadingAngle';
            saveVarsTmp{1}(3, 1).Complexity = 'real';
            saveVarsTmp{1}(3, 1).Dimensions = 1;
            saveVarsTmp{1}(3, 1).DataType = 'single';
            saveVarsTmp{1}(3, 1).Min = [];
            saveVarsTmp{1}(3, 1).Max = [];
            saveVarsTmp{1}(3, 1).DimensionsMode = 'Fixed';
            saveVarsTmp{1}(3, 1).SamplingMode = 'Sample based';
            saveVarsTmp{1}(3, 1).SampleTime = -1;
            saveVarsTmp{1}(3, 1).DocUnits = 'rad';
            saveVarsTmp{1}(3, 1).Description = '';
            saveVarsTmp{1}(4, 1) = Simulink.BusElement;
            saveVarsTmp{1}(4, 1).Name = 'LateralOffset';
            saveVarsTmp{1}(4, 1).Complexity = 'real';
            saveVarsTmp{1}(4, 1).Dimensions = 1;
            saveVarsTmp{1}(4, 1).DataType = 'single';
            saveVarsTmp{1}(4, 1).Min = [];
            saveVarsTmp{1}(4, 1).Max = [];
            saveVarsTmp{1}(4, 1).DimensionsMode = 'Fixed';
            saveVarsTmp{1}(4, 1).SamplingMode = 'Sample based';
            saveVarsTmp{1}(4, 1).SampleTime = -1;
            saveVarsTmp{1}(4, 1).DocUnits = 'm';
            saveVarsTmp{1}(4, 1).Description = '';
            saveVarsTmp{1}(5, 1) = Simulink.BusElement;
            saveVarsTmp{1}(5, 1).Name = 'Strength';
            saveVarsTmp{1}(5, 1).Complexity = 'real';
            saveVarsTmp{1}(5, 1).Dimensions = 1;
            saveVarsTmp{1}(5, 1).DataType = 'single';
            saveVarsTmp{1}(5, 1).Min = [];
            saveVarsTmp{1}(5, 1).Max = [];
            saveVarsTmp{1}(5, 1).DimensionsMode = 'Fixed';
            saveVarsTmp{1}(5, 1).SamplingMode = 'Sample based';
            saveVarsTmp{1}(5, 1).SampleTime = -1;
            saveVarsTmp{1}(5, 1).DocUnits = '';
            saveVarsTmp{1}(5, 1).Description = '';
            saveVarsTmp{1}(6, 1) = Simulink.BusElement;
            saveVarsTmp{1}(6, 1).Name = 'XExtent';
            saveVarsTmp{1}(6, 1).Complexity = 'real';
            saveVarsTmp{1}(6, 1).Dimensions = [1 2];
            saveVarsTmp{1}(6, 1).DataType = 'single';
            saveVarsTmp{1}(6, 1).Min = [];
            saveVarsTmp{1}(6, 1).Max = [];
            saveVarsTmp{1}(6, 1).DimensionsMode = 'Fixed';
            saveVarsTmp{1}(6, 1).SamplingMode = 'Sample based';
            saveVarsTmp{1}(6, 1).SampleTime = -1;
            saveVarsTmp{1}(6, 1).DocUnits = '';
            saveVarsTmp{1}(6, 1).Description = '';
            saveVarsTmp{1}(7, 1) = Simulink.BusElement;
            saveVarsTmp{1}(7, 1).Name = 'BoundaryType';
            saveVarsTmp{1}(7, 1).Complexity = 'real';
            saveVarsTmp{1}(7, 1).Dimensions = 1;
            saveVarsTmp{1}(7, 1).DataType = 'Enum: LaneBoundaryType';
            saveVarsTmp{1}(7, 1).Min = []; 
            saveVarsTmp{1}(7, 1).Max = [];
            saveVarsTmp{1}(7, 1).DimensionsMode = 'Fixed';
            saveVarsTmp{1}(7, 1).SamplingMode = 'Sample based';
            saveVarsTmp{1}(7, 1).SampleTime = -1;
            saveVarsTmp{1}(7, 1).DocUnits = '';
            saveVarsTmp{1}(7, 1).Description = '';
            LaneSensorBoundaries.Elements = saveVarsTmp{1};
            clear saveVarsTmp;


            % Create BusController
            BusController = Simulink.Bus;
            BusController.Description = '';
            BusController.DataScope = 'Auto';
            BusController.HeaderFile = '';
            BusController.Alignment = -1;
            BusController.PreserveElementDimensions = false;
            saveVarsTmp{1} = Simulink.BusElement;
            saveVarsTmp{1}.Name = 'aeb_test_enable';
            saveVarsTmp{1}.Complexity = 'real';
            saveVarsTmp{1}.Dimensions = 1;
            saveVarsTmp{1}.DataType = 'boolean';
            saveVarsTmp{1}.Min = [];
            saveVarsTmp{1}.Max = [];
            saveVarsTmp{1}.DimensionsMode = 'Fixed';
            saveVarsTmp{1}.SamplingMode = 'Sample based';
            saveVarsTmp{1}.DocUnits = '';
            saveVarsTmp{1}.Description = '';
            saveVarsTmp{1}(2, 1) = Simulink.BusElement;
            saveVarsTmp{1}(2, 1).Name = 'lka_test_enable';
            saveVarsTmp{1}(2, 1).Complexity = 'real';
            saveVarsTmp{1}(2, 1).Dimensions = 1;
            saveVarsTmp{1}(2, 1).DataType = 'boolean';
            saveVarsTmp{1}(2, 1).Min = [];
            saveVarsTmp{1}(2, 1).Max = [];
            saveVarsTmp{1}(2, 1).DimensionsMode = 'Fixed';
            saveVarsTmp{1}(2, 1).SamplingMode = 'Sample based';
            saveVarsTmp{1}(2, 1).DocUnits = '';
            saveVarsTmp{1}(2, 1).Description = '';
            saveVarsTmp{1}(3, 1) = Simulink.BusElement;
            saveVarsTmp{1}(3, 1).Name = 'BusRefPathInfo';
            saveVarsTmp{1}(3, 1).Complexity = 'real';
            saveVarsTmp{1}(3, 1).Dimensions = 1;
            saveVarsTmp{1}(3, 1).DataType = 'Bus: BusRefPathInfo';
            saveVarsTmp{1}(3, 1).Min = [];
            saveVarsTmp{1}(3, 1).Max = [];
            saveVarsTmp{1}(3, 1).DimensionsMode = 'Fixed';
            saveVarsTmp{1}(3, 1).SamplingMode = 'Sample based';
            saveVarsTmp{1}(3, 1).DocUnits = '';
            saveVarsTmp{1}(3, 1).Description = '';
            saveVarsTmp{1}(4, 1) = Simulink.BusElement;
            saveVarsTmp{1}(4, 1).Name = 'BusActorPose';
            saveVarsTmp{1}(4, 1).Complexity = 'real';
            saveVarsTmp{1}(4, 1).Dimensions = 1;
            saveVarsTmp{1}(4, 1).DataType = 'Bus: BusActorPose';
            saveVarsTmp{1}(4, 1).Min = [];
            saveVarsTmp{1}(4, 1).Max = [];
            saveVarsTmp{1}(4, 1).DimensionsMode = 'Fixed';
            saveVarsTmp{1}(4, 1).SamplingMode = 'Sample based';
            saveVarsTmp{1}(4, 1).DocUnits = '';
            saveVarsTmp{1}(4, 1).Description = '';
            saveVarsTmp{1}(5, 1) = Simulink.BusElement;
            saveVarsTmp{1}(5, 1).Name = 'BusMultiObjectTracker1';
            saveVarsTmp{1}(5, 1).Complexity = 'real';
            saveVarsTmp{1}(5, 1).Dimensions = 1;
            saveVarsTmp{1}(5, 1).DataType = 'Bus: BusMultiObjectTracker1';
            saveVarsTmp{1}(5, 1).Min = [];
            saveVarsTmp{1}(5, 1).Max = [];
            saveVarsTmp{1}(5, 1).DimensionsMode = 'Fixed';
            saveVarsTmp{1}(5, 1).SamplingMode = 'Sample based';
            saveVarsTmp{1}(5, 1).DocUnits = '';
            saveVarsTmp{1}(5, 1).Description = '';
            saveVarsTmp{1}(6, 1) = Simulink.BusElement;
            saveVarsTmp{1}(6, 1).Name = 'set_velocity';
            saveVarsTmp{1}(6, 1).Complexity = 'real';
            saveVarsTmp{1}(6, 1).Dimensions = 1;
            saveVarsTmp{1}(6, 1).DataType = 'double';
            saveVarsTmp{1}(6, 1).Min = [];
            saveVarsTmp{1}(6, 1).Max = [];
            saveVarsTmp{1}(6, 1).DimensionsMode = 'Fixed';
            saveVarsTmp{1}(6, 1).SamplingMode = 'Sample based';
            saveVarsTmp{1}(6, 1).DocUnits = '';
            saveVarsTmp{1}(6, 1).Description = '';
            saveVarsTmp{1}(7, 1) = Simulink.BusElement;
            saveVarsTmp{1}(7, 1).Name = 'BusVision';
            saveVarsTmp{1}(7, 1).Complexity = 'real';
            saveVarsTmp{1}(7, 1).Dimensions = 1;
            saveVarsTmp{1}(7, 1).DataType = 'Bus: BusVision';
            saveVarsTmp{1}(7, 1).Min = [];
            saveVarsTmp{1}(7, 1).Max = [];
            saveVarsTmp{1}(7, 1).DimensionsMode = 'Fixed';
            saveVarsTmp{1}(7, 1).SamplingMode = 'Sample based';
            saveVarsTmp{1}(7, 1).DocUnits = '';
            saveVarsTmp{1}(7, 1).Description = '';
            saveVarsTmp{1}(8, 1) = Simulink.BusElement;
            saveVarsTmp{1}(8, 1).Name = 'LaneBus';
            saveVarsTmp{1}(8, 1).Complexity = 'real';
            saveVarsTmp{1}(8, 1).Dimensions = 1;
            saveVarsTmp{1}(8, 1).DataType = 'LaneBus';
            saveVarsTmp{1}(8, 1).Min = [];
            saveVarsTmp{1}(8, 1).Max = [];
            saveVarsTmp{1}(8, 1).DimensionsMode = 'Fixed';
            saveVarsTmp{1}(8, 1).SamplingMode = 'Sample based';
            saveVarsTmp{1}(8, 1).DocUnits = '';
            saveVarsTmp{1}(8, 1).Description = '';
            BusController.Elements = saveVarsTmp{1};
            clear saveVarsTmp;


            % Create BusRefPathInfo
            BusRefPathInfo = Simulink.Bus;
            BusRefPathInfo.Description = '';
            BusRefPathInfo.DataScope = 'Auto';
            BusRefPathInfo.HeaderFile = '';
            BusRefPathInfo.Alignment = -1;
            BusRefPathInfo.PreserveElementDimensions = false;
            saveVarsTmp{1} = Simulink.BusElement;
            saveVarsTmp{1}.Name = 'BusDeviations';
            saveVarsTmp{1}.Complexity = 'real';
            saveVarsTmp{1}.Dimensions = 1;
            saveVarsTmp{1}.DataType = 'Bus: BusDeviations';
            saveVarsTmp{1}.Min = [];
            saveVarsTmp{1}.Max = [];
            saveVarsTmp{1}.DimensionsMode = 'Fixed';
            saveVarsTmp{1}.SamplingMode = 'Sample based';
            saveVarsTmp{1}.DocUnits = '';
            saveVarsTmp{1}.Description = '';
            saveVarsTmp{1}(2, 1) = Simulink.BusElement;
            saveVarsTmp{1}(2, 1).Name = 'ref_pose';
            saveVarsTmp{1}(2, 1).Complexity = 'real';
            saveVarsTmp{1}(2, 1).Dimensions = [1 3];
            saveVarsTmp{1}(2, 1).DataType = 'double';
            saveVarsTmp{1}(2, 1).Min = [];
            saveVarsTmp{1}(2, 1).Max = [];
            saveVarsTmp{1}(2, 1).DimensionsMode = 'Fixed';
            saveVarsTmp{1}(2, 1).SamplingMode = 'Sample based';
            saveVarsTmp{1}(2, 1).DocUnits = '';
            saveVarsTmp{1}(2, 1).Description = '';
            BusRefPathInfo.Elements = saveVarsTmp{1};
            clear saveVarsTmp;

            % Create BusVehicleCommands
            BusVehicleCommands = Simulink.Bus;
            BusVehicleCommands.Description = '';
            BusVehicleCommands.DataScope = 'Auto';
            BusVehicleCommands.HeaderFile = '';
            BusVehicleCommands.Alignment = -1;
            BusVehicleCommands.PreserveElementDimensions = false;
            saveVarsTmp{1} = Simulink.BusElement;
            saveVarsTmp{1}.Name = 'steering_angle';
            saveVarsTmp{1}.Complexity = 'real';
            saveVarsTmp{1}.Dimensions = 1;
            saveVarsTmp{1}.DataType = 'double';
            saveVarsTmp{1}.Min = [];
            saveVarsTmp{1}.Max = [];
            saveVarsTmp{1}.DimensionsMode = 'Fixed';
            saveVarsTmp{1}.SamplingMode = 'Sample based';
            saveVarsTmp{1}.DocUnits = '';
            saveVarsTmp{1}.Description = '';
            saveVarsTmp{1}(2, 1) = Simulink.BusElement;
            saveVarsTmp{1}(2, 1).Name = 'acceleration';
            saveVarsTmp{1}(2, 1).Complexity = 'real';
            saveVarsTmp{1}(2, 1).Dimensions = 1;
            saveVarsTmp{1}(2, 1).DataType = 'double';
            saveVarsTmp{1}(2, 1).Min = [];
            saveVarsTmp{1}(2, 1).Max = [];
            saveVarsTmp{1}(2, 1).DimensionsMode = 'Fixed';
            saveVarsTmp{1}(2, 1).SamplingMode = 'Sample based';
            saveVarsTmp{1}(2, 1).DocUnits = '';
            saveVarsTmp{1}(2, 1).Description = '';
            BusVehicleCommands.Elements = saveVarsTmp{1};
            clear saveVarsTmp;

            AccelSteerBus = Simulink.Bus;
            AccelSteerBus.Description = '';
            AccelSteerBus.DataScope = 'Auto';
            AccelSteerBus.HeaderFile = '';
            AccelSteerBus.Alignment = -1;
            AccelSteerBus.PreserveElementDimensions = false;
            saveVarsTmp{1} = Simulink.BusElement;
            saveVarsTmp{1}.Name = 'accel';
            saveVarsTmp{1}.Complexity = 'real';
            saveVarsTmp{1}.Dimensions = 1;
            saveVarsTmp{1}.DataType = 'double';
            saveVarsTmp{1}.Min = [];
            saveVarsTmp{1}.Max = [];
            saveVarsTmp{1}.DimensionsMode = 'Fixed';
            saveVarsTmp{1}.SamplingMode = 'Sample based';
            saveVarsTmp{1}.DocUnits = '';
            saveVarsTmp{1}.Description = '';
            saveVarsTmp{1}(2, 1) = Simulink.BusElement;
            saveVarsTmp{1}(2, 1).Name = 'steer';
            saveVarsTmp{1}(2, 1).Complexity = 'real';
            saveVarsTmp{1}(2, 1).Dimensions = 1;
            saveVarsTmp{1}(2, 1).DataType = 'double';
            saveVarsTmp{1}(2, 1).Min = [];
            saveVarsTmp{1}(2, 1).Max = [];
            saveVarsTmp{1}(2, 1).DimensionsMode = 'Fixed';
            saveVarsTmp{1}(2, 1).SamplingMode = 'Sample based';
            saveVarsTmp{1}(2, 1).DocUnits = '';
            saveVarsTmp{1}(2, 1).Description = '';
            AccelSteerBus.Elements = saveVarsTmp{1};
            clear saveVarsTmp;



            assignin('base','BusDeviations', BusDeviations);
            assignin('base','AccelSteerBus', AccelSteerBus);
            assignin('base','BusVehicleCommands', BusVehicleCommands);
            assignin('base','BusRefPathInfo', BusRefPathInfo);
            assignin('base','BusController', BusController);
            assignin('base','BusActorPoses', BusActorPoses);
            assignin('base','BusVehiclePose', BusVehiclePose);
            assignin('base','BusEgoRefPath', BusEgoRefPath);
            assignin('base','BusRefPath', BusRefPath);
            assignin('base','BusPathTarget', BusPathTarget);
            assignin('base','BusPathPointData', BusPathPointData);
            assignin('base','LaneBus', LaneBus);
            assignin('base','slBus1_LaneBoundaries', slBus1_LaneBoundaries);
            assignin('base','BusLaneSensor', BusLaneSensor);
            assignin('base','LaneSensorBoundaries', LaneSensorBoundaries);
            assignin('base','LaneSensor', LaneSensor);
            assignin('base','BusMultiObjectTracker1Tracks', BusMultiObjectTracker1Tracks);
            assignin('base','BusMultiObjectTracker1', BusMultiObjectTracker1);
            assignin('base','BusVisionDetectionsObjectAttributes', BusVisionDetectionsObjectAttributes);
            assignin('base','BusVisionDetectionsMeasurementParameters', BusVisionDetectionsMeasurementParameters);
            assignin('base','BusVisionDetections', BusVisionDetections);
            assignin('base','BusVision', BusVision);
            assignin('base','BusRadarDetectionsObjectAttributes', BusRadarDetectionsObjectAttributes);
            assignin('base','BusRadarDetectionsMeasurementParameters', BusRadarDetectionsMeasurementParameters);
            assignin('base','BusRadarDetections', BusRadarDetections);
            assignin('base','BusRadar', BusRadar);
        end
    end
end
