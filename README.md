# 7763-Code-2026
FRC 7763 robot code for 2026 REBUILT. Architecture and contribution rules: see [CLAUDE.md](CLAUDE.md).

## Driver controls (Xbox, port 0)

| Input | Action |
|---|---|
| Left stick / right stick X | Drive (field-relative) / rotate |
| Right bumper (hold) | Shoot: flywheel spins up, feeds once ready; drive slows to 0.5 m/s |
| B (hold) | Reverse everything: intake, floor, kicker and flywheel run backwards |
| Left bumper (hold) / A (hold) | Intake in / intake reverse |
| Left trigger / right trigger | Rack out / rack in |
| X | Rack half out |
| Y (hold) | Agitate: rack shakes while held, returns to where it was on release |
| D-pad up/down | Flywheel speed offset ±2 rps |
| D-pad left/right | Turret aim offset ∓2° |
| Start | Re-home hood (drives down until it stalls, then zeroes) |
| Back | Reset heading (robot facing away from driver) |

The turret and hood always auto-aim at the hub (or a pass target when past our trench line) while enabled.

Controller rumble: short buzz per ball shot, light buzz when ready to shoot, strong buzz 3 s before our hub turns on.

Test mode: A/B/X/Y run SysId characterization for the mechanism picked in "SysId Mechanism" (see CLAUDE.md).

## Dashboard (Elastic: load `elastic-layout.json`)

- **Driver**: shot mode (SCORING / PASSING / TRENCH), ball in shooter, ready, hub active + countdown, robot health, camera status, field, auto chooser.
- **Shots**: MARK MADE / MARK MISSED after each volley (builds `shots.csv` on the robot), flywheel current graph, toggles for Wait For Ready, Hold Fire When Hub Off, Tilt Compensation.
- **Tuning**: every live-tunable number; PRINT VALUES prints them to the console to copy into `Constants`.

Regenerate the layout after changing dashboard keys: `python tools/make_elastic_layout.py`.

## Hardware

| Mechanism | Motors (CAN ID) | Bus |
|---|---|---|
| Swerve (WCP SwerveX Flipped, X3-10) | drive 1/3/7/5, steer 2/4/8/6, CANcoders 15/12/13/14, Pigeon 10 | Drivetrain CANivore |
| Turret | 21 | Drivetrain CANivore |
| Flywheel | 22 (left), 23 (right) | rio |
| Kicker | 24 | rio |
| Hood (Kraken X44) | 25 | rio |
| Floor / indexer | 30 | rio |
| Rack (rack and pinion) | 41, 43 | rio |
| Intake rollers | 42 | rio |

Network: roboRIO 10.77.63.2, PhotonVision coprocessor 10.77.63.11 (must NOT be .2).
