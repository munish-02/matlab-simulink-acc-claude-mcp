Adaptive Cruise Control in Simulink, Built via MATLAB MCP Server + Claude

    Building a classical constant-time-gap Adaptive Cruise Control (ACC) model from scratch in Simulink, 
    entirely through natural-language prompts to Claude Desktop, using the official MathWorks MATLAB MCP Server. 
    Includes the full block architecture, validated simulation results, and an honest look at the MCP server's current drawbacks.

    Overview

    What Is the MATLAB MCP Server

    Prerequisites Checked Before Starting

    The Prompt Used

    Model Architecture

    Simulation Results

    Validation Against Theory

    Drawbacks and Limitations of the MCP Server

    Lessons Learned

    References

Overview

This repository documents building a simplified Adaptive Cruise Control (ACC) system in Simulink, generated entirely by Claude Desktop through the official MATLAB MCP Server from MathWorks, connected to a local MATLAB R2024b installation.

Unlike a toy example, ACC is a real closed-loop control problem: 
it must track a set cruise speed when the road is clear, automatically switch to gap-following when a slower lead vehicle is present, 
respect physical actuation limits, and never allow a collision. 
This makes it a much more meaningful stress test of AI-assisted, prompt-driven model-based design than a simple counter or signal generator.

The model uses only base Simulink blocks — no Automated Driving Toolbox, no Model Predictive Control Toolbox — deliberately, to keep the example runnable on any standard MATLAB/Simulink installation.
What Is the MATLAB MCP Server

The MATLAB MCP Server is MathWorks' official, open-source implementation of the Model Context Protocol (MCP), allowing AI applications like Claude Desktop, GitHub Copilot, and Claude Code to:
Tool	Description	Type
detect_matlab_toolboxes	Lists installed MATLAB version and toolboxes	Read-only
check_matlab_code	Static analysis / linting of a .m file	Read-only
evaluate_matlab_code	Executes a string of MATLAB code	Write/Execute
run_matlab_file	Executes a .m script	Write/Execute
run_matlab_test_file	Executes MATLAB unit tests	Write/Execute

It supports MATLAB R2021a and later, and connects via a locally-run server process rather than a cloud service — the AI never sees your license or data beyond what a given tool call returns.

    For the base setup, extension installation, and a full debugging log of the issues encountered getting this pipeline running on Windows, see the companion guide: matlab-simulink-claude-mcp-guide

Prerequisites Checked Before Starting

Before attempting a model this complex, the following was verified:

    Toolbox availability: ran detect_matlab_toolboxes first to confirm whether Model Predictive Control Toolbox was installed. 
    It was not, so the prompt explicitly requested a classical PID-based approach using only base Simulink, rather than the official Adaptive Cruise Control System MPC block.

    Scope decision: chose a simplified two-vehicle longitudinal model (ego + lead car) rather than a full sensor-fusion/3D-visualization build (which requires Automated Driving Toolbox and radar/vision sensor blocks) — 
    a more realistic target for a single AI-driven build session.

    Tool permissions: evaluate_matlab_code and run_matlab_file were kept on "always ask" so every generated script could be reviewed before execution.

    Safety requirement: explicitly required a collision-detection assertion in the prompt, since a plain PID model will not stop itself if gains are poorly tuned and the ego vehicle rear-ends the lead vehicle in simulation.

The Prompt Used

Using MATLAB, build a simplified Adaptive Cruise Control (ACC) Simulink model from scratch, without using the Automated Driving Toolbox or MPC Toolbox — use only base Simulink blocks, since I want a classical control approach.

Model two vehicles longitudinally:
1. A lead vehicle with a prescribed velocity profile (e.g., starts at 20 m/s, then steps down to 12 m/s at t=15s to simulate braking).
2. An ego (following) vehicle controlled by a PID controller that adjusts acceleration to:
   - Track a desired speed (default 25 m/s) when no lead vehicle is close, and
   - Maintain a safe following distance (time-gap based, e.g., 1.5 s headway) when a lead vehicle is present.

Use Integrator blocks for position/velocity from acceleration, a Saturation block to limit acceleration to realistic values (-5 to 2 m/s^2), and a Sum block to compute relative distance between the two vehicles.

Add Scope blocks to visualize: (1) ego vs lead vehicle velocity over time, (2) relative distance between vehicles over time, and (3) the ego vehicle's commanded acceleration.

Include a Stop block or assertion that halts simulation if relative distance drops below 0 (collision).

Save the model as ACC_Model.slx in my MATLAB_Claude folder. 
Then run a 30-second simulation and open the scopes so I can see the ego vehicle's speed tracking behavior and the following distance over time.

Model Architecture

Claude built the model using the classical constant-time-gap ACC structure, entirely with base Simulink blocks:

Lead vehicle:

    Step block (20 → 12 m/s at t = 15 s) → Integrator → lead position

Ego vehicle control logic:

    Cruise reference: Constant (25 m/s)

    Gap reference: Relative Distance (Sum: x_lead − x_ego) → Gap Error (vs. desired distance = Min Standstill Gap d0 (10 m) + Time Headway (1.5 s) × v_ego) → Gap Control Gain (0.6) → added to v_lead → produces the Velocity From Gap signal

    Reference Velocity Min block (min-select): picks whichever is lower — cruise speed or gap-implied speed. This is the core arbitration logic that makes the controller automatically switch between cruising and gap-following.

    Velocity Error → PID(s) (ACC PID Controller) → Acceleration Saturation (−5 to 2 m/s²) → Ego Velocity (Integrator) → Ego Position (Integrator)

Safety layer:

    Relative Distance → Collision Check GE0 (Compare To Zero, ≥ 0) → Collision Assertion block with "stop on failure" enabled — halts the simulation immediately if the ego vehicle would collide with the lead vehicle.

Visualization:

    Velocity Scope (Ego vs. Lead, via Velocity Mux)

    Distance Scope (Relative Distance)

    Acceleration Scope (Ego Command)

Model file: ACC_Model.slx, saved to the working MATLAB project folder.
Simulation Results

30-second simulation, no collision:
Phase	Ego Settled Speed	Ego–Lead Gap	Theoretical Gap (10 + 1.5·v)
Cruise-following (before brake)	~20.5 m/s	~41 m	40 m
After lead brakes at t = 15 s	12.0 m/s	~28 m	28 m (exact match)

Minimum relative distance over the entire run: 28.2 m — comfortably clear of collision, with the Collision Assertion never triggering.
Validation Against Theory

The constant-time-gap law is:


desired_distance = standstill_gap + time_gap × v_ego
                  = 10 + 1.5 × v_ego

    At 20.5 m/s: 10 + 1.5 × 20 = 40 m → simulated ≈ 41 m (small residual PID steady-state error, expected without an integral-specific gap loop)

    At 12.0 m/s: 10 + 1.5 × 12 = 28 m → simulated = 28 m (exact match)

This confirms two things worked correctly, not just that the model "ran without erroring":

    The gap-following law itself is implemented correctly.

    The min-select arbitration (cruise vs. gap-implied speed) genuinely switches behavior rather than blending the two references — this is the defining mechanism that makes this adaptive cruise control rather than plain fixed-speed cruise control.

Drawbacks and Limitations of the MCP Server

Building something as involved as ACC surfaced real limitations worth documenting honestly, beyond the basic setup issues covered in the companion repo:

    No toolbox awareness until you check. 
    The server does not warn you upfront that a requested block (e.g., the official Adaptive Cruise Control System MPC block) requires a toolbox you don't have. 
    You must proactively run detect_matlab_toolboxes yourself and steer the prompt accordingly, or the agent may generate code that fails at runtime with a licensing error.

    No native visualization return path. 
    The MCP tools return text/console output, not images. 
    Scopes open as separate MATLAB windows on the local desktop — the AI cannot "see" the resulting plot itself to confirm correctness. 
    All visual validation of the results (this README's numbers, the settling behavior, the gap accuracy) had to be verified by a human reading MATLAB's console output and scope windows directly, not by the AI inspecting the output.

    Multi-step builds require many approval prompts. 
    Because evaluate_matlab_code and run_matlab_file are (rightly) gated behind manual approval, a model this complex triggers several sequential approval dialogs rather than one atomic operation. 
    This is a deliberate safety tradeoff, but it makes rapid iteration slower than working directly in the MATLAB IDE for anyone not comfortable rubber-stamping every step.

    No built-in unit or magnitude sanity-checking. 
    The server executes exactly what it's told; it does not flag physically implausible values (e.g., an acceleration limit in the wrong units, or a time-gap value that would produce negative desired distance at high speed). 
    Reviewing generated parameter values against real-world expectations remains entirely the user's responsibility.

    Session state can be lost. 
    Depending on matlab-session-mode configuration, a new MATLAB session may spin up per tool call rather than reusing an existing one, 
    meaning variables or open figures from a previous step are not guaranteed to persist unless shareMATLABSession() was explicitly configured beforehand.

    Local-only, single-user by license. 
    Per MathWorks' own licensing terms, MCP servers must not be shared by multiple users and require a locally installed, licensed MATLAB — this is not a cloud/team-shared tool without separate arrangement with MathWorks.

    Setup friction is nontrivial. 
    As documented in the companion setup guide, getting the surrounding Claude Desktop extensions (particularly Filesystem) working reliably involved several rounds of debugging — 
    this is a newer integration path (MCP Core Server first released October 2025) and not yet as polished as MathWorks' longer-established tooling.

None of these are fatal, but they meaningfully shape how you should use this: 
as an assistant that accelerates first-draft model construction and boilerplate, with a human still required to validate physical correctness, 
check toolbox dependencies in advance, and review generated code before every execution step.

Lessons Learned

    Specify constraints explicitly. 
    Stating "no Automated Driving Toolbox or MPC Toolbox — base Simulink only" upfront avoided a failed build; without this, the agent might default to the official MPC-based ACC block regardless of what's actually licensed.

    Ask for the theory-matching validation, not just "does it run.
    " Requesting the settled gap and comparing it against 10 + 1.5·v by hand is what actually confirmed correctness — a model that runs without error is not the same as a model that behaves correctly.

    Require a safety exit condition explicitly. 
    The Collision Assertion block was only added because the prompt specifically asked for it — it is not a default safety net the agent adds on its own.

    Visual results still need a human. 
    The AI cannot see the scope output; verifying the actual traces (smooth deceleration, no oscillation, no overshoot into collision range) requires opening the scope windows yourself.

    Complex builds mean more approvals, not fewer. 
    Budget for reviewing several sequential code-execution approvals rather than expecting one clean, atomic generation step.

References

    Official MCP Server: matlab/matlab-mcp-server

    MathWorks: Adaptive Cruise Control System Using MPC

    MathWorks: Adaptive Cruise Control with Sensor Fusion

    Model Context Protocol specification

    Companion setup guide: matlab-simulink-claude-mcp-guide

License

This guide and example code are shared under the MIT License. 
The MATLAB MCP Server itself is subject to the MathWorks Software License Agreement.

## RoadRunner driving test bench

The ACC model is also integrated with a generated `drivingTestBench`, camera
and radar sensing, 3DOF vehicle dynamics, and the RoadRunner scenario
`ACC_LeadBrake_Test.rrscenario`. See
[DrivingTestProject/DrivingTest/ACC_ROADRUNNER.md](DrivingTestProject/DrivingTest/ACC_ROADRUNNER.md)
for the architecture, portable RoadRunner project, one-command test, and
verified results. The repository bundle can be refreshed with:

```matlab
manifest = packageACCRoadRunnerProject;
```

To run the complete test after cloning the repository:

```matlab
repoRoot = pwd;
project = openProject(fullfile(repoRoot, ...
    "DrivingTestProject", "DrivingTest", "DrivingTest.prj"));
rrProjectFile = fullfile(project.RootFolder, ...
    "RoadRunnerProject", "Project", "Project.rrproj");
rrProjectFolder = fileparts(fileparts(rrProjectFile));

rrApp = roadrunner(ProjectFolder=rrProjectFolder);
results = runACCLeadBrakeTest(rrApp, rrProjectFile);
```

`runACCLeadBrakeTest` accepts either the RoadRunner project folder or
`Project.rrproj`. Omitting that argument uses the project currently open in
`rrApp`.

Before pushing, stage the generated MATLAB project and portable RoadRunner
bundle. Generated simulation caches are excluded by `.gitignore`.

```powershell
git add .gitattributes .gitignore README.md DrivingTestProject
git commit -m "Add reproducible RoadRunner ACC test bench"
git push
```
