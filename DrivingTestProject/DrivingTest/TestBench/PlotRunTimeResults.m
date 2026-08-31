classdef PlotRunTimeResults < matlab.System
    %PlotRunTimeResults Helps in visualizing the run time results
    %   using MATLAB figure and birdsEyePlot.
    %
    % NOTE: The name of this System Object and it's functionality may
    % change without notice in a future release, or the System Object
    % itself may be removed.
    %

    %   Copyright 2024 The MathWorks, Inc.

    properties(Nontunable)
        %CameraParams Camera Sensor Parameters
        Camera = struct('ImageSize',[768 1024],'PrincipalPoint',[512 384],...
            'FocalLength',[512 512],'Position',[1.8750 0 1.2000],...
            'Rotation', [0,0,0], 'FieldOfView', [45,45],...
            'DetectionRanges',[6 50]);
        
        %RadarParams Radar Sensor Parameters
        Radar = struct('Position', [ 2.2, 0, 0.1 ],'Rotation', [0 0 0], ...
            'DetectionRanges',[1 150], 'FieldOfView', [120,4], ...
            'AzRes', 4, 'RangeRes', 2.5, 'RangeRateRes', 0.5);

        % Ego Car ID
        EgoCarID = 1;

        % ActorProfiles
        ActorProfiles = struct(...
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

        % Enable Display
        IsDisplayEnabled (1,1) logical = false;

        %Sample Time
        SampleTime=0.03;


    end


    % Private properties of the System object
    properties(Access = private)
        % Figure holds the instance of MATLAB figure and their properties.
        Figure;

        % BirdsEyePlot holds the instance of birdsEyePlot().
        BirdsEyePlot;

        % OutlinePlotter holds the instance of outlinePlotter().
        OutlinePlotter;

        % LaneMarkingPlotter holds the instance of LaneMarkingPlotter().
        LaneMarkingPlotter;

        % TrackPlotter holds the instance of trackPlotter
        TrackPlotter;
        tPlotter;

        % VisionDetPlotter holds the instance of vision plotter
        VisionDetPlotter;

        %Scenario Store scenario variable from the workspace.
        Scenario;

        % Color code blue for ego vehicle vehicle
        ColorBlue  = [0 0.447 0.741];


        PosSelector = [1,0,0,0,0,0; 0,0,1,0,0,0]; % Position selector   (N/A)
        VelSelector = [0,1,0,0,0,0; 0,0,0,1,0,0];
    end

    %----------------------------------------------------------------------
    % Main algorithm
    %----------------------------------------------------------------------
    methods(Access = protected)
        %------------------------------------------------------------------
        function setupImpl(obj)
            %setupImpl Initializes the properties of System object.
            % The setupImpl method initializes the Figure, BirdsEyePlot
            if(obj.IsDisplayEnabled == 1)
                % Get the scenario object from the base workspace.
                obj.Scenario = evalin('base', 'scenario');

                % % Create and set Figure Properties
                figureName = 'Run Time Visualization Plot';
                scrsz = double(get(groot,'ScreenSize'));
                obj.Figure = findobj('Type','Figure','Name',figureName);
                if isempty(obj.Figure)
                    obj.Figure = uifigure('Name',figureName);
                    obj.Figure.Position = [10 10 scrsz(3)*.8 scrsz(4)*0.8];
                end
                hax = axes(obj.Figure);

                % Create birds eye plot
                obj.BirdsEyePlot = birdsEyePlot('Parent', hax,...
                    'XLimits', [0, 160],...
                    'YLimits', [-130, 130]);

                % Create outline plotter for ego vehicle
                egoOutlinePlotter = outlinePlotter(obj.BirdsEyePlot);

                cap_radar = coverageAreaPlotter(obj.BirdsEyePlot,'FaceColor','red','EdgeColor','red');
                cap_vision = coverageAreaPlotter(obj.BirdsEyePlot,'FaceColor','blue','EdgeColor','blue');

                plotCoverageArea(cap_radar, [obj.Radar.Position(1), obj.Radar.Position(2)], obj.Radar.DetectionRanges(2), obj.Radar.Rotation(3), obj.Radar.FieldOfView(1));

                intrinsics = cameraIntrinsics(obj.Camera.FocalLength, obj.Camera.PrincipalPoint, obj.Camera.ImageSize);
                monoCamConfig = monoCamera(intrinsics, obj.Camera.Position(3),'Pitch',obj.Camera.Rotation(2));
                visionSensor = visionDetectionGenerator(monoCamConfig);
                plotCoverageArea(cap_vision, [obj.Camera.Position(1) obj.Camera.Position(2)] , obj.Camera.DetectionRanges(2), obj.Camera.Position(3), visionSensor.FieldOfView(1));

                % Plot the ego car in visualization
                plotOutline(egoOutlinePlotter,...
                    [0,0], 0, obj.ActorProfiles(obj.EgoCarID).Length, obj.ActorProfiles(obj.EgoCarID).Width,...
                    'OriginOffset', [obj.ActorProfiles(obj.EgoCarID).OriginOffset(1) obj.ActorProfiles(obj.EgoCarID).OriginOffset(2)],...
                    'Color', obj.ColorBlue);

                % Create lane marking plotter
                obj.LaneMarkingPlotter = ...
                    laneMarkingPlotter(obj.BirdsEyePlot,...
                    'DisplayName','Lane boundaries');

                % Create a new detection plotter
                obj.VisionDetPlotter = detectionPlotter(obj.BirdsEyePlot, ...
                    'DisplayName','Vision detections', ...
                    'MarkerFaceColor','b');
                
                % Create a new tracks plotter
                obj.tPlotter = trackPlotter(obj.BirdsEyePlot,'DisplayName','Tracks');

                % Create outline plotter for target vehicles
                obj.OutlinePlotter = outlinePlotter(obj.BirdsEyePlot);
            end
        end

        function flag = isInputSizeMutableImpl(~,index)
            % Return false if input size cannot change
            % between calls to the System object
            flag = (index == 5) || (index == 6);
        end

        function sts = getSampleTimeImpl(obj)
            % Define sample time type and parameters
            sts = obj.createSampleTime("Type", "Discrete", ...
                "SampleTime", obj.SampleTime);
        end

        %------------------------------------------------------------------
        function stepImpl(obj, targetActors, egoActor, tracks, visionDetections)
            %stepImpl implements the core logic for visualization at every
            %simulation step
            if ~isempty(obj.Figure) && (obj.IsDisplayEnabled == true)
                % Plot lane boundaries on the birds eye plot.
                % Update road boundaries and their display
                egoCar = evalin('base', "scenario.Actors("+ num2str(obj.EgoCarID) +")");

                % Update ego actor information
                egoCar.Position = egoActor.Position;
                egoCar.Yaw = egoActor.Yaw;
                egoCar.Roll = egoActor.Roll;
                egoCar.Pitch = egoActor.Pitch;
                % Set current velocity
                egoCar.Velocity = egoActor.Velocity;
                egoCar.AngularVelocity = egoActor.AngularVelocity;

                [lmv, lmf] = laneMarkingVertices(egoCar);
                if ~isempty(obj.LaneMarkingPlotter)
                    plotLaneMarking(obj.LaneMarkingPlotter, lmv, lmf);
                end

                % Number of actors must remain the same between steps
                numActors = targetActors.NumActors;

                % add ego car
                numActors = numActors + 1;

                positions = zeros(numActors,2);
                yaws      = zeros(numActors,1);
                lengths      = zeros(numActors,1);
                widths      = zeros(numActors,1);
                originOffsets      = zeros(numActors,2);

                % Add ego vehicle at origin
                positions(1,:) = [0,0];
                yaws(1) = 0;
                lengths(1,:) = obj.ActorProfiles(obj.EgoCarID).Length;
                widths(1,:) = obj.ActorProfiles(obj.EgoCarID).Width;
                originOffsets(1,:) = obj.ActorProfiles(obj.EgoCarID).OriginOffset(1:2);
                actorIds = ~(obj.EgoCarID == [obj.Scenario.Actors.ActorID]);

                targetActorProfiles = obj.ActorProfiles(actorIds);

                % Plot other target vehicles on birds eye plot.
                for n = 2:numActors
                    positions(n,:) = targetActors.Actors(n-1).Position(1:2);
                    yaws(n,:)      = targetActors.Actors(n-1).Yaw;
                    lengths(n,:)   = targetActorProfiles(n-1).Length;
                    widths(n,:)    = targetActorProfiles(n-1).Width;
                    originOffsets(n,:) = targetActorProfiles(n-1).OriginOffset(:,1:2);
                end

                % Plot target vehicles on birds eye plot
                if ~isempty(obj.OutlinePlotter)
                    plotOutline(obj.OutlinePlotter,...
                        positions, yaws, lengths, widths,...
                        'OriginOffset',originOffsets);
                end
                % Plot Tracks
                numTracks = tracks.NumTracks;
                trackPositions1(1,:) = [inf,inf,inf];
                trackVelocity1(1,:) = [0,0,0];
                trackIds1(1,:) = uint32(1);
                for n = 1:numTracks
                    trackPositions1(n,:) = [obj.PosSelector*tracks.Tracks(n).State; 0]';
                    trackVelocity1(n,:)  = [obj.VelSelector*tracks.Tracks(n).State; 0]';
                    trackIds1(n,:)  = tracks.Tracks(n).TrackID;
                end
                trackLabels = cellfun(@num2str, num2cell(trackIds1), 'UniformOutput', false);

                plotTrack(obj.tPlotter,trackPositions1,trackVelocity1,trackLabels);


                valid_vision_detections = [inf inf];

                for i = 1:visionDetections.NumDetections
                    Vdet = visionDetections.Detections(i).Measurement(1:2)';
                    valid_vision_detections = vertcat(valid_vision_detections, Vdet);
                end

                if(~isempty(valid_vision_detections))
                    %plot vision detections
                    plotDetection(obj.VisionDetPlotter, valid_vision_detections);
                end
            end
        end
    end

    %----------------------------------------------------------------------
    % Simulink dialog
    %----------------------------------------------------------------------
    methods(Access = protected, Static)
        %------------------------------------------------------------------
        function simMode = getSimulateUsingImpl
            % Return only allowed simulation mode in System block dialog
            simMode = "Interpreted execution";
        end

        %------------------------------------------------------------------
        function flag = showSimulateUsingImpl
            % Return false if simulation mode hidden in System block dialog
            flag = false;
        end
    end

end

