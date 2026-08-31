# ACC RoadRunner integration

This driving project connects the repository's classical adaptive cruise
controller to a generated `drivingTestBench`.

## Architecture

- Bundled RoadRunner project: `RoadRunnerProject`
- Original development project: `D:\Work\26a\checkShirt\checkShort`
- Scene: `SixLaneHighway.rrscene`
- Scenario: `ACC_LeadBrake_Test.rrscenario`
- Ego behavior: `DrivingTestBench`
- Sensors: camera and radar detections fused by the generated tracker
- Vehicle: generated 3DOF vehicle dynamics
- Steering: generated trajectory-following controller
- Longitudinal control: `ACC Longitudinal Controller` in `Controller.slx`
- Safety override: generated AEB mode selector remains connected

The ACC selects the closest tracked vehicle in the ego lane and applies:

```text
desired gap = 10 m + 1.5 s * ego speed
gap gain   = 0.6 1/s
PI gains   = Kp 1.0, Ki 0.5
limits     = -5 to 2 m/s^2
```

The test starts both vehicles at 20 m/s with a 50 m center gap. The ego cruise
request is 25 m/s. At 15 s, the lead vehicle brakes to 12 m/s. The scenario
ends at 30 s and has a collision fail condition.

## Portable RoadRunner project

The repository contains a dependency-complete RoadRunner project in
`RoadRunnerProject`. It includes only the files needed by this test, rather
than the unrelated content in the original RoadRunner workspace.

To refresh the bundle from the currently connected RoadRunner project:

```matlab
manifest = packageACCRoadRunnerProject;
```

The packager also accepts either a RoadRunner project folder or its project
file. If no argument is supplied, it uses the project connected through
`rrApp`; if that is unavailable, it defaults to
`D:\Work\26a\checkShirt\checkShort`.

```matlab
manifest = packageACCRoadRunnerProject( ...
    "D:\Work\26a\checkShirt\checkShort\Project\Project.rrproj");
```

`RoadRunnerProject/BUNDLE_MANIFEST.csv` records each packaged file, its size,
and SHA-256 checksum.

## Run from a clone

Open `DrivingTest.prj` first so that the generated Simulink models are on the
MATLAB path. Then launch RoadRunner with the bundled project:

```matlab
projectRoot = currentProject().RootFolder;
rrProjectFile = fullfile(projectRoot, "RoadRunnerProject", ...
    "Project", "Project.rrproj");
rrProjectFolder = fileparts(fileparts(rrProjectFile));

rrApp = roadrunner(ProjectFolder=rrProjectFolder);
results = runACCLeadBrakeTest(rrApp, rrProjectFile);
```

The test runner accepts either the project folder or `Project.rrproj`. When
the project argument is omitted, it defaults to the project currently open
in `rrApp`:

Open `DrivingTest.prj`, connect to the RoadRunner project above, and run:

```matlab
results = runACCLeadBrakeTest(rrApp);
```

The function deliberately reopens the scenario, prepares RoadRunner, and only
then configures the test bench. This order is required for reliable repeated
vision/radar runs in R2026a.

The latest verified result was:

| Metric | Value |
|---|---:|
| Duration | 30.000 s |
| Samples | 1,001 |
| Ego speed, initial / maximum / final | 20.000 / 23.521 / 12.062 m/s |
| Lead final speed | 12.000 m/s |
| Center gap, initial / minimum / final | 50.000 / 28.953 / 28.953 m |
| Theoretical final gap | 28.000 m |
| Result | PASS |

The function returns the metrics and sampled signals, and leaves `rrSim`,
`accSimLog`, and `accTestResults` in the base workspace for inspection.

## Source control

The portable RoadRunner project is approximately 21 MiB and has no individual
file larger than the common Git hosting limit of 100 MiB. Generated MATLAB,
Simulink, and RoadRunner output folders are excluded by the repository
`.gitignore`.

From the repository root:

```powershell
git add .gitattributes .gitignore README.md DrivingTestProject
git commit -m "Add reproducible RoadRunner ACC test bench"
git push
```
