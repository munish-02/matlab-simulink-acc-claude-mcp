function results = runACCLeadBrakeTest(rrApp, roadRunnerProject, options)
%RUNACCLEADBRAKETEST Run and verify the RoadRunner ACC braking scenario.
%
%   RESULTS = RUNACCLEADBRAKETEST(RRAPP) runs ACC_LeadBrake_Test in the
%   currently open RoadRunner project using the generated DrivingTestBench,
%   vision and radar detections, and the 3DOF vehicle.
%
%   RESULTS = RUNACCLEADBRAKETEST(RRAPP, PROJECT) accepts either a
%   RoadRunner project folder or its Project/Project.rrproj file. The
%   project is opened when it is not already active.
%
%   The function leaves the ScenarioSimulation object and log in the base
%   workspace as rrSim and accSimLog for interactive inspection.

arguments
    rrApp (1, 1) roadrunner
    roadRunnerProject {mustBeTextScalar} = ""
    options.DesiredSpeed (1, 1) double {mustBePositive} = 25
    options.StepSize (1, 1) double {mustBeBetween(options.StepSize, 0.01, 0.05)} = 0.03
    options.StopTime (1, 1) double {mustBePositive} = 30
    options.WallClockTimeout (1, 1) double {mustBePositive} = 180
end

targetScenario = "ACC_LeadBrake_Test.rrscenario";

rrStatus = status(rrApp);
currentProject = replace(string(rrStatus.Project.Filename), "\", "/");
projectWasSpecified = strlength(string(roadRunnerProject)) > 0;
if ~projectWasSpecified
    targetProject = stripTrailingSlash(currentProject);
else
    targetProject = replace(string(resolveProjectRoot(roadRunnerProject)), "\", "/");
end

if projectWasSpecified
    assertNoUnsavedRoadRunnerChanges(rrStatus);
    openProject(rrApp, targetProject);
    rrStatus = status(rrApp);
end

if ~isempty(rrStatus.Scenario) && isstruct(rrStatus.Scenario) ...
        && rrStatus.Scenario.UnsavedChanges
    error("ACC:UnsavedScenario", ...
        "Save or discard the current RoadRunner scenario changes before running the test.");
end

clearPreviousSimulation();

% Reopening resets actor state and profiles for reliable repeated runs.
openScenario(rrApp, targetScenario);
prepareSimulation(rrApp);

rrSim = createSimulation(rrApp);
set(rrSim, "Logging", "On");
set(rrSim, "StepSize", options.StepSize);
set(rrSim, "MaxSimulationTime", options.StopTime);

vehicleConfig = configureDrivingTestBench.VehicleParameters(Fidelity="3DOF");
cfg = configureDrivingTestBench;
cfg.setup(Controller="Trajectory Follower", ...
    ObjectDetections="vision-radar", ...
    Vehicle=vehicleConfig, ...
    StepSize=options.StepSize, ...
    EnableVisualization=false);
assignin("base", "steadyStateSpeed", options.DesiredSpeed);
assignin("base", "rrSim", rrSim);

set(rrSim, "SimulationCommand", "Start");
waitForSimulation(rrSim, options.WallClockTimeout);

simLog = get(rrSim, "SimulationLog");
poseEgo = get(simLog, "Pose", "ActorID", uint64(1));
poseLead = get(simLog, "Pose", "ActorID", uint64(2));
velocityEgo = get(simLog, "Velocity", "ActorID", uint64(1));
velocityLead = get(simLog, "Velocity", "ActorID", uint64(2));

assert(~isempty(poseEgo) && ~isempty(poseLead), ...
    "ACC:EmptyLog", "RoadRunner returned an empty actor log.");
assert(numel(poseEgo) == numel(poseLead) ...
    && numel(poseEgo) == numel(velocityEgo) ...
    && numel(poseEgo) == numel(velocityLead), ...
    "ACC:LogSizeMismatch", "Actor pose and velocity logs have different lengths.");

egoPositionCells = arrayfun(@(sample) sample.Pose(1:3, 4)', ...
    poseEgo, UniformOutput=false);
leadPositionCells = arrayfun(@(sample) sample.Pose(1:3, 4)', ...
    poseLead, UniformOutput=false);
egoPosition = vertcat(egoPositionCells{:});
leadPosition = vertcat(leadPositionCells{:});
egoVelocity = vertcat(velocityEgo.Velocity);
leadVelocity = vertcat(velocityLead.Velocity);

time = vertcat(poseEgo.Time);
egoSpeed = vecnorm(egoVelocity, 2, 2);
leadSpeed = vecnorm(leadVelocity, 2, 2);
centerGap = vecnorm(leadPosition - egoPosition, 2, 2);

standstillGap = evalin("base", "accStandstillGap");
timeHeadway = evalin("base", "accTimeHeadway");
targetFinalGap = standstillGap + timeHeadway * leadSpeed(end);

checks = struct;
checks.CompletedScenario = time(end) >= options.StopTime - options.StepSize;
checks.EgoAccelerated = max(egoSpeed) > egoSpeed(1) + 0.5;
checks.LeadBraked = abs(leadSpeed(end) - 12) < 0.5;
checks.EgoMatchedLead = abs(egoSpeed(end) - leadSpeed(end)) < 0.5;
checks.NoVehicleOverlap = min(centerGap) > 8;
checks.FinalGapTracked = abs(centerGap(end) - targetFinalGap) < 3;

results = struct;
results.Passed = all(structfun(@(value) logical(value), checks));
results.Checks = checks;
results.Duration = time(end);
results.NumSamples = numel(time);
results.InitialEgoSpeed = egoSpeed(1);
results.MaximumEgoSpeed = max(egoSpeed);
results.FinalEgoSpeed = egoSpeed(end);
results.FinalLeadSpeed = leadSpeed(end);
results.InitialCenterGap = centerGap(1);
results.MinimumCenterGap = min(centerGap);
results.FinalCenterGap = centerGap(end);
results.TargetFinalGap = targetFinalGap;
results.Signals = table(time, egoSpeed, leadSpeed, centerGap);

assignin("base", "accSimLog", simLog);
assignin("base", "accTestResults", results);

fprintf("ACC RoadRunner verification: %s\n", passFail(results.Passed));
fprintf("  Duration: %.3f s (%d samples)\n", ...
    results.Duration, results.NumSamples);
fprintf("  Ego speed initial / max / final: %.3f / %.3f / %.3f m/s\n", ...
    results.InitialEgoSpeed, results.MaximumEgoSpeed, results.FinalEgoSpeed);
fprintf("  Lead final speed: %.3f m/s\n", results.FinalLeadSpeed);
fprintf("  Center gap initial / min / final / target: %.3f / %.3f / %.3f / %.3f m\n", ...
    results.InitialCenterGap, results.MinimumCenterGap, ...
    results.FinalCenterGap, results.TargetFinalGap);

assert(results.Passed, "ACC:VerificationFailed", ...
    "The ACC RoadRunner scenario did not satisfy every acceptance check.");
end

function clearPreviousSimulation()
if ~evalin("base", "exist('rrSim', 'var')")
    return
end

previousSimulation = evalin("base", "rrSim");
simulationStatus = string(get(previousSimulation, "SimulationStatus"));
if ismember(simulationStatus, ["Running", "Paused"])
    error("ACC:SimulationActive", ...
        "A RoadRunner simulation is active. Stop it before starting this test.");
end

delete(previousSimulation);
evalin("base", "clear rrSim");
end

function waitForSimulation(rrSim, wallClockTimeout)
startTime = tic;
simulationStatus = string(get(rrSim, "SimulationStatus"));
while ~ismember(simulationStatus, ["Done", "Inactive", "Stopped"])
    pause(0.25);
    if toc(startTime) > wallClockTimeout
        set(rrSim, "SimulationCommand", "Stop");
        error("ACC:SimulationTimeout", ...
            "RoadRunner did not finish within %.1f wall-clock seconds.", ...
            wallClockTimeout);
    end
    simulationStatus = string(get(rrSim, "SimulationStatus"));
end
end

function pathValue = stripTrailingSlash(pathValue)
pathValue = regexprep(pathValue, "/+$", "");
end

function projectRoot = resolveProjectRoot(projectInput)
projectInput = char(projectInput);
if isfile(projectInput)
    [projectFolder, projectName, projectExtension] = fileparts(projectInput);
    if ~strcmpi(projectName + string(projectExtension), "Project.rrproj")
        error("ACC:InvalidRoadRunnerProjectFile", ...
            "Expected a Project.rrproj file, received '%s'.", projectInput);
    end
    projectRoot = fileparts(projectFolder);
elseif isfolder(projectInput)
    if isfile(fullfile(projectInput, "Project", "Project.rrproj"))
        projectRoot = projectInput;
    elseif isfile(fullfile(projectInput, "Project.rrproj"))
        projectRoot = fileparts(projectInput);
    else
        error("ACC:InvalidRoadRunnerProjectFolder", ...
            "'%s' does not contain Project/Project.rrproj.", projectInput);
    end
else
    error("ACC:RoadRunnerProjectNotFound", ...
        "RoadRunner project does not exist: '%s'.", projectInput);
end

projectRoot = char(java.io.File(projectRoot).getCanonicalPath());
end

function assertNoUnsavedRoadRunnerChanges(rrStatus)
hasUnsavedScenario = ~isempty(rrStatus.Scenario) ...
    && isstruct(rrStatus.Scenario) ...
    && rrStatus.Scenario.UnsavedChanges;
if rrStatus.Project.UnsavedChanges || rrStatus.Scene.UnsavedChanges ...
        || hasUnsavedScenario
    error("ACC:UnsavedRoadRunnerChanges", ...
        "Save or discard current RoadRunner changes before switching projects.");
end
end

function text = passFail(passed)
if passed
    text = "PASS";
else
    text = "FAIL";
end
end
