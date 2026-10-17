# 7763 Robot Code — guide for AI agents (and humans)

This codebase is written and maintained primarily by Claude. Keep it small, flat and testable.

## Architecture (read this first)

One loop, one decision-maker, no hidden behavior:

```
buttons / auto named commands ──► Superstructure (requested modes)
drivetrain pose + velocity + tilt ──► ShotCalculator ──► Shot
Superstructure.update(time, Shot, Measured sensors) ──► Setpoints ──► Robot.apply() ──► motors
around the loop: Vision, Tuning, HubSchedule, BallDetector + ShotLog, RumbleTimer, Health
```

| File | Role | Pure (unit tested)? |
|---|---|---|
| `Constants.java` | Every number: CAN IDs, gains, speeds, field geometry, shot table | — |
| `Tuning.java` | Values editable live from the dashboard (defaults from Constants) | yes |
| `Robot.java` | Owns all hardware, button map, auto named commands, SysId, the loop, dashboard output | no |
| `Superstructure.java` | State machine for every mechanism except the drivetrain (feed gating, agitate, reverse, hood homing) | **yes** |
| `HubSchedule.java` | 2026 hub active/inactive shifts from FMS game data | **yes** |
| `RumbleTimer.java` | Controller rumble pulses | **yes** |
| `shooting/ShotCalculator.java` | Pose → turret/hood/flywheel, shoot-on-the-move, trench, passing, tilt | **yes** |
| `shooting/TiltCompensation.java` | "Shoot on the bump" math | **yes** |
| `shooting/BallDetector.java` | Ball-through-flywheel detection from current | **yes** |
| `shooting/ShotLog.java` | Per-shot records + made/missed marking → CSV | **yes** |
| `subsystems/SwerveDrive.java` | Drivetrain interface + PathPlanner setup | — |
| `subsystems/CustomSwerve.java` + `SwerveModule.java` | Our own swerve (default) | no |
| `subsystems/CtreSwerve.java` | CTRE swerve library, same constants (`Constants.Drive.USE_CTRE_SWERVE`) | no |
| `subsystems/{Turret,Hood,Flywheel,Rack,Roller}.java` | Thin motor wrappers: config, `set…()`, `log()` | no |
| `subsystems/Vision.java` | PhotonVision → pose estimator, with per-camera status text | no |
| `subsystems/Health.java` | Device/CAN/battery problems in plain English | no |
| `subsystems/MechanismSim.java` | DC-motor physics for mechanism motors in simulation | no |

Mechanism classes are **dumb**: they accept a setpoint and log. All decisions (what to run, when, timers,
homing, gating) live in the pure classes, which are fully unit tested. `RobotSimTest` boots the whole robot
in simulation and checks the shoot chain end to end.

## Rules

1. **No magic numbers outside `Constants.java`.** Name them with units (`_M`, `_DEG`, `_ROT`, `_RPS`, `_DUTY`, `_S`, `_A`).
   If the team will want to tune it on the robot, also add it to `Tuning.java` and read it with `Tuning.X.get()`.
2. **Behavior changes go in pure classes with a test.** Change `Superstructure`/`ShotCalculator`/etc → add or update a test.
3. **Run `./gradlew build` before calling anything done.** It compiles and runs every test (GitHub runs it too).
4. **Keep hardware classes thin.** No state machines, timers or game logic in `subsystems/`.
5. **Don't change the controller map or auto named-command names** (`Shoot_5s`, `Shoot_Off`, `Intake_On`, `Intake_Off`,
   `Rack_Extend`, `Rack_Retract`, `Rack_{80,70,60,50}pct`, `Agitate`) without being asked. The .auto/.path files depend on them.
6. **Coordinates:** poses are blue-origin (WPILib convention). `ShotCalculator` flips into "own alliance" frame
   (180° rotation, 2026 field is rotationally symmetric). Turret angles are robot-relative, CCW+, 0 = forward.
7. **Dashboard:** driver-facing values go under `SmartDashboard` `Driver/...`; everything else via `DogLog.log()`.
   After adding/renaming a dashboard key or a Tuning value, run `python tools/make_elastic_layout.py`.
8. New CTRE devices must go through `Talons.create(name, ...)` (or `Health.register`) so health checks see them.
9. Code that only runs on real hardware (configs, inversions, CAN IDs) can't be tested here. Say so explicitly
   when a change needs on-robot verification.

## Boot assumptions (encoders are zeroed at power-on)

- Turret pointing straight forward. Hood fully down (or press **Start** to re-home). Rack fully retracted.

## Data workflows

- **CarboVis match view:** the log keys it reads are specified in
  https://github.com/Carrborobotics/CarboVis/blob/main/logging-needed/match-view-logging.md (source of truth; edit it
  there). Don't rename or change the type/units/sign of `Vision/<cam>/...`, `Drive/Pose`, `Drive/ModuleAngles`,
  `DS/...`, `System/LoopTimeMs`, `Motors/<name>/...` or `Mechanisms/...` keys without checking that file first.

- **Shot table from real shots:** drive + shoot, press MARK MADE / MARK MISSED on the Shots tab after each volley.
  Copy `/home/lvuser/shots.csv` off the robot (`scp lvuser@10.77.63.2:shots.csv .`) and run
  `python tools/analyze_shots.py shots.csv` for a suggested `SHOT_TABLE`.
- **Live tuning:** edit values on the Tuning tab, press PRINT VALUES, copy the printed lines into `Constants`.
- **SysId:** test mode, choose "SysId Mechanism", hold A/B/X/Y (quasistatic fwd/rev, dynamic fwd/rev). Load the
  roboRIO's `.wpilog` in the WPILib SysId tool; put the resulting kS/kV/kA/kP into `Constants`.

## Commands

- `./gradlew build` — compile + all tests
- `./gradlew simulateJava` — desktop sim (mechanisms have physics; balls are simulated while feeding)
- `./gradlew deploy` — deploy to the robot (team number from `.wpilib/wpilib_preferences.json`)
