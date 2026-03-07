package frc.robot;

import java.util.EnumMap;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
//import frc.robot.Constants.Localization.ReefFace;
import frc.robot.commands.LocalSwerve;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.rack.Rack;
import frc.robot.subsystems.rack.RackIOReal;
import frc.robot.subsystems.rack.RackIOSim;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOReal;
import frc.robot.subsystems.intake.IntakeIOSim;
// import frc.robot.subsystems.led.LedSubsystem;
// import frc.robot.subsystems.led.LedSubsystem.LedMode;
//import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretIOReal;
import frc.robot.subsystems.turret.TurretIOSim;   
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIOReal;
import frc.robot.subsystems.shooter.ShooterIOSim;   
import frc.robot.subsystems.floor.Floor;
import frc.robot.subsystems.floor.FloorIOReal;
import frc.robot.subsystems.floor.FloorIOSim;   

import static edu.wpi.first.units.Units.Degrees;
import frc.robot.Constants;

public class RobotContainer {
    /* Auto */
    private final SendableChooser<Command> autoChooser;

    /* Controllers */
    CommandXboxController driverController = new CommandXboxController(0);

    /* Drive Controls */
    private final Supplier<Double> translationAxis = driverController::getLeftY;
    private final Supplier<Double> strafeAxis = driverController::getLeftX;
    private final Supplier<Double> rotationAxis = driverController::getRightX;
    private final Supplier<Double> turretAxis = () ->
        driverController.getRightTriggerAxis() - driverController.getLeftTriggerAxis();

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Intake intake;
    private final Rack rack;
    private final Floor floor;
    private final Shooter shooter;
    private final Turret turret;

    // private final LedSubsystem m_led = new LedSubsystem();
    private final Field2d targetField;

    public Swerve getSwerve() {
        return s_Swerve;
    }

    /* Alliance colors */
    private final Color redBumper = Color.kDarkRed;
    private final Color blueBumper = Color.kDarkBlue;
    private Color original_color;

    public RobotContainer() {
        if (Robot.isReal()) {
            this.rack = new Rack(new RackIOReal());
            this.intake = new Intake(new IntakeIOReal());
            this.floor = new Floor(new FloorIOReal());
            this.shooter = new Shooter(new ShooterIOReal());
            this.turret = new Turret(new TurretIOReal());
        } else {
            this.rack = new Rack(new RackIOSim()); // Simulated rack for testing
            this.intake = new Intake(new IntakeIOSim()); // Simulated intake for testing
            this.floor = new Floor(new FloorIOSim());
            this.shooter = new Shooter(new ShooterIOSim());
            this.turret = new Turret(new TurretIOSim());
        }

        targetField = new Field2d();
        SmartDashboard.putData("Target Field", targetField);

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                    s_Swerve,
                    () -> -translationAxis.get(),
                    () -> -strafeAxis.get(),
                    () -> -rotationAxis.get(),
                    () -> false,
                    () -> false
            )
        );

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        driverController.povUp().onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
        driverController.povDown().onTrue(s_Swerve.resetModulesToAbsolute());

        // Turret rotation with Triggers
        turret.setDefaultCommand(
            Commands.run(() -> {
                double raw = turretAxis.get();
                if (Math.abs(raw) < 0.05) return;
                turret.setTurretAngle(raw * 135.0);
            }, turret)
        );

        // rack goes out (deployed)
        driverController.leftTrigger().onTrue(rack.rackToCmd(-31400.0));
        // rack goes in (retracted)
        driverController.rightTrigger().onTrue(rack.rackToCmd(0.0));

        // these dont really work because rack is in position control mode.
        // we may consider having a mode for speed control for manual override later??
        driverController.leftBumper().whileTrue(rack.setSpeedCmd(0.1)).onFalse(rack.setSpeedCmd(0));
        driverController.rightBumper().whileTrue(rack.setSpeedCmd(-0.1)).onFalse(rack.setSpeedCmd(0));

        // run the intake
        //driverController.rightBumper().whileTrue(intake.setIntakeSpeed(0.25)).onFalse(intake.stopCmd());

        // turret testing
        driverController.a().onTrue(turret.turretToCmd(360.0));
        driverController.b().onTrue(turret.turretToCmd(0.0));
        driverController.x().onTrue(turret.turretToCmd(180.0));
        driverController.y().onTrue(turret.turretToCmd(355.0));
        //driverController.y().onTrue(turret.turretTo(-5.0));
        
        // Hold left bumper to auto-aim at hub, release to go back to trigger control
        driverController.leftBumper().whileTrue(
            Commands.run(() -> turret.setTurretAngle(getAngleToHub()), turret)
        );
    }

    // ── Targeting ──────────────────────────────────────────────────────────────
    private double getAngleToHub() {
        Pose2d robotPose = s_Swerve.getPose();
        Translation2d hub = Swerve.flipIfRed(Constants.Localization.hubPosition);
        Translation2d toHub = robotPose.getTranslation().minus(hub);
        double fieldAngle = Math.toDegrees(Math.atan2(toHub.getY(), toHub.getX()));
        double turretAngle = fieldAngle - robotPose.getRotation().getDegrees();
        turretAngle = MathUtil.inputModulus(turretAngle + 180, -180, 180);
        return MathUtil.clamp(turretAngle, -135.0, 135.0);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected()
            .deadlineWith(Commands.run(() -> turret.setTurretAngle(getAngleToHub()), turret));
    }

    public Turret getTurret() {
        return turret;
    }
}