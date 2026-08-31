classdef TargetBehavior < matlab.System
    %TargetBehavior is used to observe VUT and implement target trajectory
    % in RoadRunner Scenario.
    %
    % NOTE: The name of this System Object and it's functionality may
    % change without notice in a future release, or the System Object
    % itself may be removed.

    % Copyright 2024 The MathWorks, Inc.

    properties(Access = private)
        % ActorSimulation object
        ActorObj

        % ScenarioSimulation object
        ScenarioSimulationObj

        % Simulation step size
        StepSize

        % Check if steady state is reached
        GoalReached = false;

        % Frequency at steady state
        NumStepsAtSteadyState = 0;

        % Target waypoint time adapter
        WaypointAdapter

        % Target Pose estimator
        ActorPoseEstimator

        % Target RefPath timing data
        RefPathTimingData

        % Ego RefPath timing data
        EgoRefPathTimingData

        % Ego Actor ID
        EgoActorID = 1;
        EgoActorDim;

        % Steady State Speed of Ego
        EgoSteadyStateSpeed

        % Initialize target flag
        TargetInitialized;

        % Distance
        EgoDistance = 0;
        EgoSteadyStateDistance = 0;
        PrevDistanceDiff = 10000;

        % Target Steady State Index
        TargetActorDim;
        TargetSteadyStateIdx
        TargetSteadyStateTime = 0;
        TargetSpeed;

        % Stationary target flag
        StationaryTarget=false;

        % Trimmed Path
        TrimmedPath
        TrimmedNumPoints
        InitialPositionYaw

        % Collision point on the path
        EgoDistanceToCollision;
        TargetDistanceToCollision;
        CurrentEgoDistance = 0;
        CurrentTargetDistance = 0;

        % Continuous tracking flag
        ContinuousTracking = 0;

        % Previous pose of the actor
        PrevPose = eye(4);
    end

    properties
        % Speed Threshold
        SpeedTolerance = 0.28;

        % Minimum steps at steady state
        MinStepsAtSteadyState = 3;

        % Synchronize Collision for junction scenarios
        SynchronizeCollision = 0;

        % Synchronization stop distance in case of continuous tracking
        StopSyncDistance = 50;
    end

    methods(Access = protected)
        function setupImpl(obj)
            % Find the ActorSimulation object of the runtime actor
            obj.ActorObj = Simulink.ScenarioSimulation.find('ActorSimulation', 'SystemObject', obj);

            % Get the ScenarioSimulation object.
            obj.ScenarioSimulationObj = Simulink.ScenarioSimulation.find('ScenarioSimulation', 'SystemObject', obj);

            % Get EgoID from world actor, getScenario
            worldActor = obj.ScenarioSimulationObj.getScenario();
            world = worldActor.actor_spec.world_spec;

            % Find ego ActorID
            for i = 1:length(world.behaviors)
                if(~isempty(world.behaviors(i).asset_reference))
                    if contains(upper(world.behaviors(i).asset_reference),upper('DrivingTestBench.rrbehavior'))
                        egoBehavior = world.behaviors(i).id;
                        for j = 1:length(world.actors)
                            id = str2double(world.actors(j).actor_spec.id);
                            if isequal(world.actors(j).actor_spec.behavior_id,egoBehavior)
                                obj.EgoActorID = id;
                            end
                        end
                        break;
                    end
                end
            end

            % Find Ego actor dimension
            for j = 1:length(world.actors)
                id = str2double(world.actors(j).actor_spec.id);
                if(obj.EgoActorID==id)
                    egoActorDim = world.actors(j).actor_spec.bounding_box;
                    break;
                end
            end

            % Find Target actor dimension
            for j = 1:length(world.actors)
                primaryTargetId = 2;
                id = str2double(world.actors(j).actor_spec.id);
                if(primaryTargetId==id)
                    targetActorDim = world.actors(j).actor_spec.bounding_box;
                    break;
                end
            end

            % Set the max value of bounding box.
            obj.EgoActorDim.max.x = egoActorDim.max.x;
            obj.EgoActorDim.max.y = egoActorDim.max.y;
            obj.EgoActorDim.max.z = egoActorDim.max.z;

            % Get the min and max values of target
            obj.TargetActorDim.max.x = targetActorDim.max.x;
            obj.TargetActorDim.max.y = targetActorDim.max.y;
            obj.TargetActorDim.max.z = targetActorDim.max.z;
            obj.TargetActorDim.min.x = targetActorDim.min.x;
            obj.TargetActorDim.min.y = targetActorDim.min.y;
            obj.TargetActorDim.min.z = targetActorDim.min.z;

            % Save Ego path timing data and ego steady state speed
            egoPathActionAdapter = drivingApplication.internal.path.PathActionAdapter;
            allActors = obj.ScenarioSimulationObj.get('ActorSimulation');
            egoPathAction = allActors{obj.EgoActorID+1}.getAction('PathAction');
            [~,obj.EgoRefPathTimingData] = egoPathActionAdapter(egoPathAction);
            obj.EgoSteadyStateSpeed = obj.EgoRefPathTimingData.speed(end-2);

            % Get Target path action and check if Target is stationary
            pathAction = obj.ActorObj.getAction('PathAction');
            if(isempty(pathAction.PathTarget.Path))
                obj.StationaryTarget = true;
            else
                % Step size
                obj.StepSize = obj.ScenarioSimulationObj.get('StepSize');

                % Process path action
                targetPath = pathAction.PathTarget.Path;
                numTargetPathPoints = pathAction.PathTarget.NumPoints;

                % Initialize waypoint time adapter and actor pose estimator
                obj.WaypointAdapter = drivingApplication.internal.path.WaypointAdapter;
                obj.ActorPoseEstimator = drivingApplication.internal.path.ActorPoseEstimator;

                % Save target path timing data
                targetPathActionAdapter = drivingApplication.internal.path.PathActionAdapter;
                [~,obj.RefPathTimingData] = targetPathActionAdapter(pathAction);

                % Find steady state speed of actor, ramp up idx, and ramp up time
                targetSteadyStateSpeed = max(obj.RefPathTimingData.speed);
                obj.TargetSteadyStateIdx = find(abs(obj.RefPathTimingData.speed - targetSteadyStateSpeed) < sqrt(eps('single')),1);

                %% Compute steady state start position of target
                speedAdapter = drivingApplication.internal.path.WaypointAdapter;
                poseEstimator = drivingApplication.internal.path.ActorPoseEstimator;
                x = 0;
                y = 0;
                z = 0;
                yaw = 0;
                currentTime = 0;
                traversedDist = 0;
                stepSize = obj.StepSize;

                % Iterate until steady state position of target is reached.
                while traversedDist < obj.RefPathTimingData.s(obj.TargetSteadyStateIdx)
                    speed = stepImpl(speedAdapter, stepSize, currentTime, obj.RefPathTimingData);
                    [x, y, z, yaw, ~, ~] = poseEstimator(targetPath, numTargetPathPoints, stepSize, speed);
                    currentTime = currentTime + stepSize;
                    traversedDist = traversedDist + stepSize * speed;
                end
                obj.TargetSteadyStateTime = currentTime;

                index = find(obj.RefPathTimingData.s > traversedDist,1);
                if traversedDist < obj.RefPathTimingData.s(index)
                    obj.TrimmedPath = [[x y z];targetPath(index:end,:)];
                else
                    obj.TrimmedPath = targetPath(index:end,:);
                    obj.TrimmedPath(1,:) = [x y z];
                end

                obj.InitialPositionYaw = yaw;
                obj.TrimmedNumPoints = size(obj.TrimmedPath,1);

                % Find Ego steady state distance using steady state time of
                % target
                obj.EgoSteadyStateDistance = interp1(obj.EgoRefPathTimingData.time, obj.EgoRefPathTimingData.s, obj.TargetSteadyStateTime);

                obj.TargetInitialized = false;
                obj.GoalReached = false;

                % Find intersection point index
                [iEgo, iTarget] = findCurveIntersections(obj,egoPathAction.PathTarget.Path(:,1), egoPathAction.PathTarget.Path(:,2), pathAction.PathTarget.Path(:,1), pathAction.PathTarget.Path(:,2));

                % If there is no intersection of trajectories, then do not synchronize target.
                if(~isempty(iEgo) && ~isempty(iTarget))
                    indexOnEgoPath = floor(iEgo);
                    indexOnTargetPath = floor(iTarget);
                    egoFraction = iEgo - floor(iEgo);
                    targetFraction = iTarget - floor(iTarget);

                    obj.EgoDistanceToCollision = obj.EgoRefPathTimingData.s(indexOnEgoPath) + egoFraction*(obj.EgoRefPathTimingData.s(indexOnEgoPath+1) - obj.EgoRefPathTimingData.s(indexOnEgoPath)) - obj.EgoActorDim.max.y;
                    obj.TargetDistanceToCollision = obj.RefPathTimingData.s(indexOnTargetPath) + targetFraction*(obj.RefPathTimingData.s(indexOnTargetPath+1)-obj.RefPathTimingData.s(indexOnTargetPath)) - traversedDist;

                    egoAngleAtCollision = obj.EgoRefPathTimingData.theta(indexOnEgoPath);
                    targetAngleAtCollision = obj.RefPathTimingData.theta(indexOnTargetPath);

                    % Update target distance to collision based on near/far
                    % and same/opposite direction of Ego
                    targetLength = obj.TargetActorDim.max.y - obj.TargetActorDim.min.y;
                    egoAngleDegree = rad2deg(egoAngleAtCollision);
                    targetAngleDegree = rad2deg(targetAngleAtCollision);

                    % Add offsets based on target
                    perpendicularPedestrian = 0;
                    offset = 0;
                    if(targetLength < 0.7) % Pedestrian
                        if(targetAngleDegree==-90 && egoAngleDegree==180)
                            perpendicularPedestrian = 1;
                        elseif (targetAngleDegree<5 && egoAngleDegree>0) % Near opposite
                            offset = -0.75;
                        elseif(targetAngleDegree<5 && egoAngleDegree<0) % Far opposite
                            offset = -0.5;
                        elseif((targetAngleDegree>=175 && targetAngleDegree<=185) && egoAngleDegree>0) % Near same
                            offset = 0.75;
                        elseif((targetAngleDegree>=175 && targetAngleDegree<=185) && egoAngleDegree<0) % Far same
                            offset = 0.5;
                        end
                    elseif (targetLength >1.0 && targetLength<2) % Bicyclist
                        if(targetAngleDegree<5 && egoAngleDegree<0) % Far
                            offset = -2.279;
                        elseif (targetAngleDegree<5 && egoAngleDegree>0) % Near
                            angleDiff =  abs(targetAngleDegree-egoAngleDegree);
                            if (angleDiff>=85  && angleDiff<=95) || (angleDiff>=265  && angleDiff<=275)
                                offset = 0.4;
                            else
                                offset = -2.5;
                            end
                        end
                    elseif (targetLength >2.0 && targetLength<2.2) % Motorcyclist
                        offset = -3.15;
                    elseif (targetLength > 3.5) % GVT
                        offset = 0.911;
                        obj.ContinuousTracking = 1;
                    end

                    obj.TargetDistanceToCollision = obj.TargetDistanceToCollision  + offset;

                    % If target and ego orientation difference is not at 0 or 180 degrees, then SynchronizeCollision.
                    angleDiff = abs(wrapTo180(rad2deg(abs(egoAngleAtCollision-targetAngleAtCollision))));
                    if(angleDiff<5 || (angleDiff>=175 && angleDiff<=185)|| perpendicularPedestrian==1)
                        obj.SynchronizeCollision = 0;
                    else
                        obj.SynchronizeCollision = 1;
                    end
                else
                    obj.SynchronizeCollision = 0;
                end

            end
        end

        function stepImpl(obj)
            if(~obj.StationaryTarget)
                % Get all vehicle actorRuntime
                currentTime = getCurrentTime(obj);
                allActorRuntime = getAllActorRuntime(obj);

                % Observe when Ego reaches steady state speed and get the corresponding position.
                currentEgoVelocity = allActorRuntime(obj.EgoActorID+1).Velocity;
                currentEgoSpeed = norm(currentEgoVelocity);

                if(obj.GoalReached == false)
                    obj.EgoDistance = obj.EgoDistance + currentEgoSpeed * obj.StepSize;

                    if(abs(currentEgoSpeed-obj.EgoSteadyStateSpeed)<obj.SpeedTolerance)
                        obj.NumStepsAtSteadyState = obj.NumStepsAtSteadyState + 1;
                    else
                        obj.NumStepsAtSteadyState = 0;
                    end

                    if(obj.NumStepsAtSteadyState > obj.MinStepsAtSteadyState)
                        currentDistanceDiff = abs(obj.EgoDistance + obj.EgoSteadyStateSpeed*obj.StepSize - obj.EgoSteadyStateDistance);


                        if(currentDistanceDiff >= obj.PrevDistanceDiff)
                            obj.GoalReached = true;
                        else
                            obj.PrevDistanceDiff = currentDistanceDiff;
                        end
                    else
                        if (obj.EgoDistance > obj.EgoSteadyStateDistance)
                            warning("VUT (Vehicle Under Test) failed to reach steady state at the initial test position." + ...
                                "Review and modify the VUT operating conditions to achieve the steady state before commencing the test.");
                        end
                    end
                end

                % Update GVT from its steady state pose only after VUT reaches
                % steady state speed.
                if(obj.GoalReached==true)
                    if(obj.TargetInitialized==false)
                        % Set target steady state time to waypoint adapter
                        % and initialize actor pose estimator.
                        setCurrentTime(obj.WaypointAdapter, obj.TargetSteadyStateTime);
                        obj.ActorPoseEstimator(obj.TrimmedPath, obj.TrimmedNumPoints, obj.StepSize, obj.RefPathTimingData.speed(obj.TargetSteadyStateIdx));

                        if(obj.SynchronizeCollision)
                            egoTime = (obj.EgoDistanceToCollision - obj.CurrentEgoDistance)/currentEgoSpeed;
                            sTarget = (obj.TargetDistanceToCollision - obj.CurrentTargetDistance);
                            nextSpeed = (sTarget)/egoTime;
                            obj.TargetSpeed = nextSpeed;
                        end
                        obj.TargetInitialized = true;
                    end


                    % Synchronize collision and update target speed
                    if(obj.SynchronizeCollision)
                        if(obj.ContinuousTracking)
                            sTarget = (obj.TargetDistanceToCollision - obj.CurrentTargetDistance);
                            if(obj.StopSyncDistance < sTarget)
                                egoTime = (obj.EgoDistanceToCollision - obj.CurrentEgoDistance)/currentEgoSpeed;
                                nextSpeed = (sTarget)/egoTime;
                                obj.TargetSpeed = nextSpeed;
                            else
                                nextSpeed = obj.TargetSpeed;
                            end
                        else
                            nextSpeed = obj.TargetSpeed;
                        end
                    else
                        nextSpeed = stepImpl(obj.WaypointAdapter, obj.StepSize, currentTime, obj.RefPathTimingData);
                    end

                    [posX, posY, posZ, yaw, ~, ~] = obj.ActorPoseEstimator(obj.TrimmedPath, obj.TrimmedNumPoints, obj.StepSize, nextSpeed);

                    % Calculate transformation matrix to write to RoadRunner
                    pose = packPose(obj, posX, posY, posZ, yaw);
                    
                    % Check if the target actor is moving. If not assign
                    % the next speed to zero.
                    if(obj.PrevPose == pose)
                        nextSpeed = 0;
                    end
                    obj.PrevPose = pose;

                    % Calculate velocity from speed and yaw
                    % Convert to RR coordinate system by changing yaw by 90
                    % degrees.
                    velocity = nextSpeed*[-sin(yaw) cos(yaw) 0];
                else
                    pose = packPose(obj, obj.TrimmedPath(1,1), obj.TrimmedPath(1,2), obj.TrimmedPath(1,3),...
                        obj.InitialPositionYaw);
                    velocity = [0 0 0];
                end

                obj.CurrentEgoDistance = obj.CurrentEgoDistance + currentEgoSpeed * obj.StepSize;
                obj.CurrentTargetDistance = obj.CurrentTargetDistance + norm(velocity) * obj.StepSize;
            else
                % Get vehicle runtime for stationary target
                vehicleRuntime = getVehicleRuntime(obj);
                pose = vehicleRuntime.ActorRuntime.Pose;
                velocity = [0 0 0];
            end
            % Update pose to RR Scenario
            obj.ActorObj.setAttribute('Pose', pose);
            obj.ActorObj.setAttribute('Velocity', velocity);
        end

        function allActorRuntime = getAllActorRuntime(obj)
            % Calculate ActorRuntime of all the vehicles
            % Get ActorSimulation object of all actors
            allActors = obj.ScenarioSimulationObj.get('ActorSimulation');
            allActorRuntime = repmat(obj.actorRuntime, length(allActors), 1);
            for i = 1:length(allActors)
                actor = allActors{i};
                allActorRuntime(i).ActorID = actor.getAttribute('ID');
                allActorRuntime(i).Pose = actor.getAttribute('Pose');
                allActorRuntime(i).Velocity = actor.getAttribute('Velocity');
                allActorRuntime(i).AngularVelocity = actor.getAttribute('AngularVelocity');
            end
        end

        function vehicleRuntime = getVehicleRuntime(obj)
            % Calculate VehicleRuntime
            actorRuntime = obj.actorRuntime;
            % Get runtime information
            actorRuntime.ActorID = obj.ActorObj.getAttribute('ID');
            actorRuntime.Pose = obj.ActorObj.getAttribute('Pose');
            actorRuntime.Velocity = obj.ActorObj.getAttribute('Velocity');
            actorRuntime.AngularVelocity = obj.ActorObj.getAttribute('AngularVelocity');
            wheelPoses = obj.ActorObj.getAttribute('WheelPoses');
            vehicleRuntime = struct('ActorRuntime', actorRuntime, ...
                'NumWheels', 4, ...
                'WheelPoses', wheelPoses);
        end

        function pose = packPose(~, posX, posY, posZ, yaw)
            % Pack the pose into 4x4 matrix.
            Position = [posX posY posZ];
            Rotation = [yaw 0 0];
            % Get rotation matrix
            rotm = robotics.internal.eul2rotm(Rotation,'ZYX');
            pose = [[rotm Position']; [0 0 0 1]];
        end

        function out = actorRuntime(~)
            %ACTORRUNTIME Creates the default actor runtime struct
            out = struct(...
                'ActorID', uint64(0), ...
                'Pose', zeros(4,4), ...
                'Velocity', zeros(1,3), ...
                'AngularVelocity', zeros(1,3));
        end

    end
    methods(Hidden)
        function [iout,jout] = findCurveIntersections(obj,curve1xpts,curve1ypts,curve2xpts,curve2ypts)

            % Check whether Curve-1 x and y points are vectors of equal
            % length with at least 2 points.
            if sum(size(curve1xpts) > 1) ~= 1 || sum(size(curve1ypts) > 1) ~= 1 || ...
                    length(curve1xpts) ~= length(curve1ypts)
                error('X and Y points of the first curve must be vectors of equal length, containing at least 2 points each.')
            end

            % Convert inputs to be column vectors.
            curve1xpts = curve1xpts(:);
            curve1ypts = curve1ypts(:);

            % Check whether Curve-2 x and y points are vectors of equal
            % length with at least 2 points.
            if sum(size(curve2xpts) > 1) ~= 1 || sum(size(curve2ypts) > 1) ~= 1 || ...
                    length(curve2xpts) ~= length(curve2ypts)
                error('X and Y points of the second curve must be vectors of equal length, containing at least 2 points each.')
            end

            % Convert inputs to be column vectors.
            curve2xpts = curve2xpts(:);
            curve2ypts = curve2ypts(:);

            % Find the number of line segments in each curve.
            curve1xy = [curve1xpts curve1ypts];
            curve2xy = [curve2xpts curve2ypts];

            % Compute differences and store for future use.
            diffCurve1xy = diff(curve1xy);
            diffCurve2xy = diff(curve2xy);

            % Find the combinations of indices, i and j where the
            % rectangle enclosing the i'th line segment of curve1 overlaps
            % with the rectangle enclosing the j'th line segment of curve2.

            % Use implicit expansion.
            [i,j] = find( ...
                mvmin(obj,curve1xpts) <= mvmax(obj,curve2xpts).' & mvmax(obj,curve1xpts) >= mvmin(obj,curve2xpts).' & ...
                mvmin(obj,curve1ypts) <= mvmax(obj,curve2ypts).' & mvmax(obj,curve1ypts) >= mvmin(obj,curve2ypts).');

            % Find and remove segments pairs which have at least one vertex
            % as NaN.
            remove = isnan(sum(diffCurve1xy(i,:) + diffCurve2xy(j,:),2));

            i(remove) = [];
            j(remove) = [];

            % Initialize matrices.
            n = length(i);
            T = zeros(4,n);
            AA = zeros(4,4,n);
            AA([1 2],3,:) = -1;
            AA([3 4],4,:) = -1;
            AA([1 3],1,:) = diffCurve1xy(i,:).';
            AA([2 4],2,:) = diffCurve2xy(j,:).';
            B = -[curve1xpts(i) curve2xpts(j) curve1ypts(i) curve2ypts(j)].';

            epsilon = 1e-10;
            for k = 1:n
                [L,U] = lu(AA(:,:,k) + epsilon * eye(4));
                T(:,k) = U\(L\B(:,k));
            end

            % Find where t1 and t2 are between 0 and 1.
            inRange = (T(1,:) >= 0 & T(2,:) >= 0 & T(1,:) < 1 & T(2,:) < 1).';

            % Compute how far along each line segment the intersections are.
            iout = i(inRange) + T(1,inRange).';
            jout = j(inRange) + T(2,inRange).';

        end

        function y = mvmin(~,x)
            y = min(x(1:end-1), x(2:end));
        end

        % Faster implementation of movmax(x,k) when k = 1.
        function y = mvmax(~,x)
            y = max(x(1:end-1), x(2:end));
        end
    end
end
