package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardOpenOption;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.CAN;
import frc.robot.Constants.Driver;
import frc.robot.Superstructure.FeedMode;
import frc.robot.Superstructure.IntakeMode;
import frc.robot.Superstructure.Measured;
import frc.robot.Superstructure.Setpoints;
import frc.robot.shooting.BallDetector;
import frc.robot.shooting.ShotCalculator;
import frc.robot.shooting.ShotCalculator.Shot;
import frc.robot.shooting.ShotLog;
import frc.robot.subsystems.CtreSwerve;
import frc.robot.subsystems.CustomSwerve;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Health;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.MechanismSim;
import frc.robot.subsystems.MotorTelemetry;
import frc.robot.subsystems.Rack;
import frc.robot.subsystems.Roller;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;

/**
 * Owns every piece of hardware and runs the loop:
 * <ol>
 *   <li>buttons / auto named commands set requested modes on the {@link Superstructure}
 *   <li>{@link ShotCalculator} turns the robot pose into a shot solution
 *   <li>the Superstructure turns modes + shot + sensor readings into {@link Setpoints}
 *   <li>{@link #apply} sends those setpoints to the motors
 * </ol>
 * Around that loop: vision, live tuning, the hub schedule, ball detection + shot log, rumble and health.
 */
public class Robot extends TimedRobot {
    private final CommandXboxController controller = new CommandXboxController(Driver.PORT);

    private final SwerveDrive drive = Constants.Drive.USE_CTRE_SWERVE ? new CtreSwerve() : new CustomSwerve();
    private final Vision vision = new Vision();
    private final Turret turret = new Turret();
    private final Hood hood = new Hood();
    private final Flywheel flywheel = new Flywheel();
    private final Roller kicker = new Roller("Kicker", CAN.KICKER, InvertedValue.Clockwise_Positive, NeutralModeValue.Coast);
    private final Roller floor = new Roller("Floor", CAN.FLOOR, InvertedValue.CounterClockwise_Positive, NeutralModeValue.Brake);
    private final Roller intake = new Roller("Intake", CAN.INTAKE, InvertedValue.CounterClockwise_Positive, NeutralModeValue.Brake);
    private final Rack rack = new Rack();

    private final Superstructure superstructure = new Superstructure();
    private final BallDetector ballDetector = new BallDetector();
    private final ShotLog shotLog = new ShotLog();
    private final RumbleTimer rumble = new RumbleTimer();
    private final SendableChooser<Command> autoChooser;
    private final SendableChooser<String> sysIdChooser = new SendableChooser<>();
    private Command autoCommand;

    // Loop-to-loop memory for edge detection.
    private double turretCommandDeg = 0;
    private int lastBallCount = 0;
    private boolean lastReady = false;
    private double lastHubChangeInS = Double.POSITIVE_INFINITY;
    private double nextHealthCheckS = 0;
    private double nextSimBallS = 0;
    private double lastLoopS = Double.NaN;

    // Dashboard keys (Elastic reads these; see elastic-layout.json).
    private static final String TILT_TOGGLE = "Driver/Tilt Compensation";
    private static final String WAIT_TOGGLE = "Driver/Wait For Ready";
    private static final String HOLD_FIRE_TOGGLE = "Driver/Hold Fire When Hub Off";
    private static final String MARK_MADE = "Driver/Mark Made";
    private static final String MARK_MISSED = "Driver/Mark Missed";
    private static final Path SHOT_FILE = Filesystem.getOperatingDirectory().toPath().resolve("shots.csv");

    public Robot() {
        // Extras: battery, brownouts, CAN utilization/errors and DS state under SystemStats/ (CarboVis match view).
        DogLog.setOptions(new DogLogOptions().withCaptureDs(true).withNtPublish(true).withLogExtras(true));
        // PDH per-channel currents, total current/power and temperature go into DogLog's SystemStats/.
        // Auto-detects a REV PDH (CAN 1) or CTRE PDP (CAN 0); if neither answers, telemetry just goes without it.
        try {
            DogLog.setPdh(new PowerDistribution());
        } catch (RuntimeException e) {
            DriverStation.reportWarning("No power distribution hub found for logging: " + e.getMessage(), false);
        }

        registerNamedCommands(); // must happen before autos are loaded
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        Tuning.publish();
        SmartDashboard.putBoolean(TILT_TOGGLE, Constants.Shooting.TILT_COMPENSATION);
        SmartDashboard.putBoolean(WAIT_TOGGLE, Constants.Feed.WAIT_FOR_READY);
        SmartDashboard.putBoolean(HOLD_FIRE_TOGGLE, Constants.Hub.HOLD_FIRE_WHEN_INACTIVE);
        SmartDashboard.putBoolean(MARK_MADE, false);
        SmartDashboard.putBoolean(MARK_MISSED, false);

        bindControls();
        bindSysId();
        CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
    }

    /**
     * Driver controls (single Xbox controller, port 0).
     *
     * <pre>
     *   Left stick     drive (field-relative)        Right stick X  rotate
     *   Right bumper   HOLD shoot (also slows drive)  B             HOLD reverse every roller + flywheel
     *   Left bumper    HOLD intake                    A             HOLD intake reverse
     *   Left trigger   rack out                       Right trigger rack in
     *   X              rack half out                  Y             HOLD agitate rack (returns when released)
     *   D-pad up/down  flywheel offset +/- 2 rps      D-pad L/R     turret offset -/+ 2 deg
     *   Start          re-home hood                   Back          reset heading (face away from driver)
     * </pre>
     * Test mode uses A/B/X/Y for SysId instead (see {@link #bindSysId}).
     */
    private void bindControls() {
        Trigger notTest = new Trigger(() -> !DriverStation.isTest());
        controller.rightBumper().and(notTest)
                .onTrue(Commands.runOnce(() -> superstructure.setFeed(FeedMode.SHOOT)))
                .onFalse(Commands.runOnce(() -> superstructure.setFeed(FeedMode.OFF)));
        controller.b().and(notTest)
                .onTrue(Commands.runOnce(() -> superstructure.setReverse(true)))
                .onFalse(Commands.runOnce(() -> superstructure.setReverse(false)));
        controller.leftBumper().and(notTest)
                .onTrue(Commands.runOnce(() -> superstructure.setIntake(IntakeMode.IN)))
                .onFalse(Commands.runOnce(() -> superstructure.setIntake(IntakeMode.OFF)));
        controller.a().and(notTest)
                .onTrue(Commands.runOnce(() -> superstructure.setIntake(IntakeMode.OUT)))
                .onFalse(Commands.runOnce(() -> superstructure.setIntake(IntakeMode.OFF)));

        controller.leftTrigger().and(notTest).onTrue(Commands.runOnce(() -> superstructure.setRack(1.0)));
        controller.rightTrigger().and(notTest).onTrue(Commands.runOnce(() -> superstructure.setRack(0.0)));
        controller.x().and(notTest).onTrue(Commands.runOnce(() -> superstructure.setRack(0.5)));
        controller.y().and(notTest)
                .onTrue(Commands.runOnce(() -> superstructure.startAgitate(Timer.getFPGATimestamp())))
                .onFalse(Commands.runOnce(superstructure::stopAgitate));

        controller.povUp().onTrue(Commands.runOnce(() -> superstructure.adjustFlywheelOffset(Driver.FLYWHEEL_OFFSET_STEP_RPS)));
        controller.povDown().onTrue(Commands.runOnce(() -> superstructure.adjustFlywheelOffset(-Driver.FLYWHEEL_OFFSET_STEP_RPS)));
        controller.povLeft().onTrue(Commands.runOnce(() -> superstructure.adjustTurretOffset(-Driver.TURRET_OFFSET_STEP_DEG)));
        controller.povRight().onTrue(Commands.runOnce(() -> superstructure.adjustTurretOffset(Driver.TURRET_OFFSET_STEP_DEG)));

        controller.start().onTrue(Commands.runOnce(() -> superstructure.startHoodHoming(Timer.getFPGATimestamp())));
        controller.back().onTrue(Commands.runOnce(() -> drive.resetHeading(isRed() ? Rotation2d.kPi : Rotation2d.kZero)));
    }

    /**
     * SysId characterization, TEST MODE ONLY. Pick "SysId Mechanism" on the dashboard, then hold:
     * A = quasistatic forward, B = quasistatic reverse, X = dynamic forward, Y = dynamic reverse.
     * Run all four, then open the roboRIO log in the WPILib SysId tool. The drive routine MOVES THE
     * ROBOT: give it several meters of clear floor and let go of the button to stop.
     */
    private void bindSysId() {
        sysIdChooser.setDefaultOption("Flywheel", "Flywheel");
        sysIdChooser.addOption("Drive", "Drive");
        SmartDashboard.putData("SysId Mechanism", sysIdChooser);

        double[] driveVolts = {0};
        SysIdRoutine driveRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(Volts.per(Second).of(Constants.SysId.DRIVE_RAMP_V_PER_S),
                        Volts.of(Constants.SysId.DRIVE_STEP_V), Seconds.of(Constants.SysId.DRIVE_TIMEOUT_S)),
                new SysIdRoutine.Mechanism(
                        v -> {
                            driveVolts[0] = v.in(Volts);
                            drive.setSysIdVolts(driveVolts[0]);
                        },
                        log -> log.motor("drive")
                                .voltage(Volts.of(driveVolts[0]))
                                .linearPosition(Meters.of(drive.sysIdPositionM()))
                                .linearVelocity(MetersPerSecond.of(drive.sysIdVelocityMps())),
                        drive));

        SubsystemBase flywheelLock = new SubsystemBase("FlywheelSysId") {};
        SysIdRoutine flywheelRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(Volts.per(Second).of(Constants.SysId.FLYWHEEL_RAMP_V_PER_S),
                        Volts.of(Constants.SysId.FLYWHEEL_STEP_V), Seconds.of(Constants.SysId.FLYWHEEL_TIMEOUT_S)),
                new SysIdRoutine.Mechanism(
                        v -> flywheel.setSysIdVolts(v.in(Volts)),
                        log -> log.motor("flywheel")
                                .voltage(Volts.of(flywheel.sysIdVolts()))
                                .angularPosition(Rotations.of(flywheel.rotations()))
                                .angularVelocity(RotationsPerSecond.of(flywheel.rps())),
                        flywheelLock));

        Trigger test = new Trigger(DriverStation::isTestEnabled);
        Set<Subsystem> requirements = Set.of(drive, flywheelLock);
        Supplier<SysIdRoutine> selected =
                () -> "Drive".equals(sysIdChooser.getSelected()) ? driveRoutine : flywheelRoutine;
        controller.a().and(test).whileTrue(Commands.defer(
                () -> selected.get().quasistatic(SysIdRoutine.Direction.kForward), requirements));
        controller.b().and(test).whileTrue(Commands.defer(
                () -> selected.get().quasistatic(SysIdRoutine.Direction.kReverse), requirements));
        controller.x().and(test).whileTrue(Commands.defer(
                () -> selected.get().dynamic(SysIdRoutine.Direction.kForward), requirements));
        controller.y().and(test).whileTrue(Commands.defer(
                () -> selected.get().dynamic(SysIdRoutine.Direction.kReverse), requirements));
    }

    /**
     * Commands the PathPlanner autos call by name. The names must match the .auto/.path files exactly.
     * Shooting is latched: "Shoot_5s" starts shooting and waits 5 s, but the robot keeps shooting until
     * "Shoot_Off" or the end of auto. (The existing autos were built around that behavior.)
     */
    private void registerNamedCommands() {
        NamedCommands.registerCommand("Shoot_5s",
                Commands.runOnce(() -> superstructure.setFeed(FeedMode.SHOOT)).andThen(Commands.waitSeconds(5)));
        NamedCommands.registerCommand("Shoot_Off", Commands.runOnce(() -> superstructure.setFeed(FeedMode.OFF)));
        NamedCommands.registerCommand("Intake_On", Commands.runOnce(() -> superstructure.setIntake(IntakeMode.IN)));
        NamedCommands.registerCommand("Intake_Off", Commands.runOnce(() -> superstructure.setIntake(IntakeMode.OFF)));
        NamedCommands.registerCommand("Rack_Extend", Commands.runOnce(() -> superstructure.setRack(1.0)));
        NamedCommands.registerCommand("Rack_Retract", Commands.runOnce(() -> superstructure.setRack(0.0)));
        for (int pct : new int[] {80, 70, 60, 50}) {
            NamedCommands.registerCommand("Rack_" + pct + "pct", Commands.runOnce(() -> superstructure.setRack(pct / 100.0)));
        }
        NamedCommands.registerCommand("Agitate",
                Commands.runOnce(() -> superstructure.startAgitate(Timer.getFPGATimestamp()))
                        .andThen(Commands.waitSeconds(Constants.Rack.AGITATE_AUTO_S))
                        .finallyDo(superstructure::stopAgitate));
    }

    // ------------------------------------------------------------------ the loop

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run(); // drivetrain odometry, auto commands, button bindings
        double now = Timer.getFPGATimestamp();
        if (!Double.isNaN(lastLoopS)) DogLog.log("System/LoopTimeMs", (now - lastLoopS) * 1000.0, "ms");
        lastLoopS = now;
        Tuning.update();
        vision.addMeasurements(drive::addVisionMeasurement);

        boolean tiltComp = SmartDashboard.getBoolean(TILT_TOGGLE, Constants.Shooting.TILT_COMPENSATION);
        Shot shot = ShotCalculator.calculate(drive.getPose(), drive.getFieldVelocity(), isRed(), drive.getTilt(), tiltComp);

        HubSchedule.Status hub = HubSchedule.status(DriverStation.getAlliance(), DriverStation.isAutonomousEnabled(),
                DriverStation.isTeleopEnabled(), DriverStation.getMatchTime(), DriverStation.getGameSpecificMessage());
        superstructure.setWaitForReady(SmartDashboard.getBoolean(WAIT_TOGGLE, Constants.Feed.WAIT_FOR_READY));
        superstructure.setHoldFire(SmartDashboard.getBoolean(HOLD_FIRE_TOGGLE, false)
                && !hub.active() && shot.mode() == ShotCalculator.Mode.SCORING);

        if (isEnabled() && !isTest()) {
            Measured measured = new Measured(hood.statorAmps(), flywheel.atSpeed(), turret.degrees() - turretCommandDeg);
            apply(superstructure.update(now, shot, measured));
        }

        ballDetector.update(now, flywheel.goalRps(), flywheel.rps(), flywheel.statorAmps());
        logShots(now, shot);
        updateRumble(now, hub);
        if (now >= nextHealthCheckS) {
            nextHealthCheckS = now + Constants.Health.CHECK_PERIOD_S;
            publishHealth();
        }
        log(shot, hub, now);
        MotorTelemetry.update();
    }

    private void apply(Setpoints sp) {
        turretCommandDeg = sp.turretDeg();
        turret.setDegrees(sp.turretDeg());
        if (sp.hoodHomingVolts() != 0) {
            hood.setVolts(sp.hoodHomingVolts());
        } else {
            if (sp.zeroHood()) hood.zero();
            hood.setUnits(sp.hoodUnits());
        }
        flywheel.setRps(sp.flywheelRps());
        kicker.set(sp.kickerDuty());
        floor.set(sp.floorDuty());
        intake.set(sp.intakeDuty());
        rack.setRotations(sp.rackRot());
    }

    @Override
    public void teleopPeriodic() {
        boolean shooting = superstructure.feedMode() == FeedMode.SHOOT;
        double maxSpeed = shooting ? Constants.Drive.SHOOTING_MAX_SPEED_MPS : Constants.Drive.MAX_SPEED_MPS;
        double maxOmega = shooting ? Constants.Drive.SHOOTING_MAX_OMEGA_RAD_PER_S : Constants.Drive.MAX_OMEGA_RAD_PER_S;

        // Stick forward = away from our driver station. Red drivers look down the field the other way.
        double alliance = isRed() ? -1 : 1;
        double x = -MathUtil.applyDeadband(controller.getLeftY(), Driver.DEADBAND) * alliance;
        double y = -MathUtil.applyDeadband(controller.getLeftX(), Driver.DEADBAND) * alliance;
        double omega = -MathUtil.applyDeadband(controller.getRightX(), Driver.DEADBAND);
        drive.driveFieldRelative(x * maxSpeed, y * maxSpeed, omega * maxOmega);
    }

    @Override
    public void simulationPeriodic() {
        MechanismSim.update();
        // Fake balls: while feeding at speed, a ball goes through the flywheel every BALL_INTERVAL_S.
        double now = Timer.getFPGATimestamp();
        if (superstructure.isFeeding() && flywheel.goalRps() > 0 && now >= nextSimBallS) {
            flywheel.simBallThrough();
            nextSimBallS = now + Constants.Sim.BALL_INTERVAL_S;
        }
    }

    @Override
    public void autonomousInit() {
        superstructure.stopAll();
        autoCommand = autoChooser.getSelected();
        if (autoCommand != null) CommandScheduler.getInstance().schedule(autoCommand);
    }

    @Override
    public void teleopInit() {
        if (autoCommand != null) autoCommand.cancel();
        superstructure.stopAll(); // don't carry auto's latched shooter/intake into teleop
    }

    @Override
    public void disabledInit() {
        superstructure.stopAll();
        writeShots(shotLog.flushUnknown());
        controller.getHID().setRumble(RumbleType.kBothRumble, 0);
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
        superstructure.stopAll();
    }

    // ------------------------------------------------------------------ helpers

    /** Record each newly detected ball, and apply Made/Missed marks from the dashboard. */
    private void logShots(double now, Shot shot) {
        if (ballDetector.ballsShot() > lastBallCount) {
            Rotation3d tilt = drive.getTilt();
            writeShots(shotLog.record(now, DriverStation.getMatchTime(), shot.mode().name(), shot.distanceM(),
                    flywheel.goalRps(), flywheel.rps(), shot.hoodUnits(), turret.degrees() - turretCommandDeg,
                    drive.getFieldVelocity().getNorm(), Math.toDegrees(Math.hypot(tilt.getX(), tilt.getY()))));
            rumble.pulse(now, Constants.Rumble.BALL_SHOT_STRENGTH, Constants.Rumble.BALL_SHOT_S);
        }
        lastBallCount = ballDetector.ballsShot();

        if (SmartDashboard.getBoolean(MARK_MADE, false)) {
            SmartDashboard.putBoolean(MARK_MADE, false);
            writeShots(shotLog.mark(true));
        }
        if (SmartDashboard.getBoolean(MARK_MISSED, false)) {
            SmartDashboard.putBoolean(MARK_MISSED, false);
            writeShots(shotLog.mark(false));
        }
    }

    private void writeShots(List<String> lines) {
        if (lines.isEmpty()) return;
        try {
            if (!Files.exists(SHOT_FILE)) Files.writeString(SHOT_FILE, ShotLog.CSV_HEADER + "\n");
            Files.write(SHOT_FILE, lines, StandardOpenOption.APPEND);
        } catch (IOException e) {
            DriverStation.reportWarning("Couldn't write " + SHOT_FILE + ": " + e.getMessage(), false);
        }
        for (String line : lines) DogLog.log("ShotLog/Line", line);
    }

    private void updateRumble(double now, HubSchedule.Status hub) {
        boolean ready = superstructure.feedMode() == FeedMode.SHOOT && superstructure.isReady();
        if (ready && !lastReady) rumble.pulse(now, Constants.Rumble.READY_STRENGTH, Constants.Rumble.READY_S);
        lastReady = ready;

        double warn = Constants.Hub.WARN_BEFORE_ACTIVE_S;
        if (hub.known() && !hub.active() && hub.secondsUntilChange() <= warn && lastHubChangeInS > warn) {
            rumble.pulse(now, Constants.Rumble.HUB_WARNING_STRENGTH, Constants.Rumble.HUB_WARNING_S);
        }
        lastHubChangeInS = hub.secondsUntilChange();

        double strength = DriverStation.isTeleopEnabled() ? rumble.strength(now) : 0;
        controller.getHID().setRumble(RumbleType.kBothRumble, strength);
    }

    private void publishHealth() {
        List<String> problems = Health.problems();
        String text = problems.isEmpty() ? "OK" : String.join(" | ", problems);
        SmartDashboard.putBoolean("Driver/Healthy", problems.isEmpty());
        SmartDashboard.putString("Driver/Robot Health", text);
        DogLog.log("Health/Problems", problems.toArray(String[]::new));
    }

    private void log(Shot shot, HubSchedule.Status hub, double now) {
        // Driver dashboard (Elastic): everything under SmartDashboard/Driver.
        SmartDashboard.putString("Driver/Shot Mode", shot.mode().name());
        SmartDashboard.putBoolean("Driver/Scoring", shot.mode() == ShotCalculator.Mode.SCORING);
        SmartDashboard.putBoolean("Driver/Passing", shot.mode() == ShotCalculator.Mode.PASSING);
        SmartDashboard.putBoolean("Driver/Trench", shot.mode() == ShotCalculator.Mode.TRENCH);
        SmartDashboard.putBoolean("Driver/Ball In Shooter", ballDetector.ballInShooter(now));
        SmartDashboard.putNumber("Driver/Balls Shot", ballDetector.ballsShot());
        SmartDashboard.putBoolean("Driver/Flywheel At Speed", flywheel.atSpeed());
        SmartDashboard.putBoolean("Driver/Ready", superstructure.isReady());
        SmartDashboard.putBoolean("Driver/Feeding", superstructure.isFeeding());
        SmartDashboard.putBoolean("Driver/Holding Fire", superstructure.isHoldingFire());
        SmartDashboard.putNumber("Driver/Flywheel Amps", flywheel.statorAmps());
        SmartDashboard.putNumber("Driver/Match Time", DriverStation.getMatchTime());
        SmartDashboard.putNumber("Driver/Flywheel Offset", superstructure.flywheelOffsetRps());
        SmartDashboard.putNumber("Driver/Turret Offset", superstructure.turretOffsetDeg());
        SmartDashboard.putString("Driver/Hood Homing", superstructure.lastHomingResult());
        SmartDashboard.putNumber("Driver/Tilt Turret Fix Deg", shot.tiltTurretDeg());
        SmartDashboard.putString("Driver/Hub", hub.label());
        SmartDashboard.putBoolean("Driver/Hub Active", hub.active());
        SmartDashboard.putNumber("Driver/Hub Change In", Double.isInfinite(hub.secondsUntilChange()) ? -1 : hub.secondsUntilChange());
        SmartDashboard.putNumber("Driver/Shots Unmarked", shotLog.unmarkedCount());
        SmartDashboard.putNumber("Driver/Shots Made", shotLog.madeCount());
        SmartDashboard.putNumber("Driver/Shots Missed", shotLog.missedCount());

        DogLog.log("Shot/Target", shot.target());
        DogLog.log("Shot/DistanceM", shot.distanceM());
        DogLog.log("Shot/TurretDeg", shot.turretDeg());
        DogLog.log("Shot/HoodUnits", shot.hoodUnits());
        DogLog.log("Shot/FlywheelRps", shot.flywheelRps());
        DogLog.log("Shot/NearTrench", shot.nearTrench());
        DogLog.log("Shot/Mode", shot.mode().name());
        DogLog.log("Shot/TiltTurretDeg", shot.tiltTurretDeg());
        DogLog.log("Shot/TiltHoodUnits", shot.tiltHoodUnits());
        DogLog.log("Shot/TiltCompensation", SmartDashboard.getBoolean(TILT_TOGGLE, false));
        DogLog.log("Shooter/BallInShooter", ballDetector.ballInShooter(now));
        DogLog.log("Shooter/BallsShot", ballDetector.ballsShot());
        DogLog.log("Shooter/DetectorArmed", ballDetector.armed());
        DogLog.log("Superstructure/Intake", superstructure.intakeMode().name());
        DogLog.log("Superstructure/Feed", superstructure.feedMode().name());
        DogLog.log("Superstructure/Ready", superstructure.isReady());
        DogLog.log("Superstructure/Feeding", superstructure.isFeeding());
        DogLog.log("Superstructure/HoldingFire", superstructure.isHoldingFire());
        DogLog.log("Superstructure/Agitating", superstructure.isAgitating());
        DogLog.log("Superstructure/Reversing", superstructure.isReversing());
        DogLog.log("Superstructure/HoodHoming", superstructure.isHoming());
        DogLog.log("Superstructure/LastHomingResult", superstructure.lastHomingResult());
        DogLog.log("Superstructure/FlywheelOffsetRps", superstructure.flywheelOffsetRps());
        DogLog.log("Superstructure/TurretOffsetDeg", superstructure.turretOffsetDeg());
        DogLog.log("Hub/Active", hub.active());
        DogLog.log("Hub/ChangeInS", hub.secondsUntilChange());
        DogLog.log("Hub/Known", hub.known());
        DogLog.log("MatchTime", DriverStation.getMatchTime());
        DogLog.log("DS/MatchTime", DriverStation.getMatchTime(), "s");
        for (int port = 0; port < Driver.LOGGED_JOYSTICK_PORTS; port++) {
            boolean connected = DriverStation.isJoystickConnected(port);
            DogLog.log("DS/Joystick" + port + "/Connected", connected);
            DogLog.log("DS/Joystick" + port + "/Name", connected ? DriverStation.getJoystickName(port) : "");
        }
        DogLog.log("IsRed", isRed());
        turret.log();
        hood.log();
        flywheel.log();
        kicker.log();
        floor.log();
        intake.log();
        rack.log();
    }

    public static boolean isRed() {
        return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red;
    }
}
