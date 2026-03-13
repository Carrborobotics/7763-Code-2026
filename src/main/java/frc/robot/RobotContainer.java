package frc.robot;

import static edu.wpi.first.units.Units.Degrees;

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
import frc.robot.subsystems.shooterhood.ShooterHood;
import frc.robot.subsystems.shooterhood.ShooterHoodIOReal;
import frc.robot.subsystems.shooterhood.ShooterHoodIOSim;
import frc.robot.subsystems.floor.Floor;
import frc.robot.subsystems.floor.FloorIOReal;
import frc.robot.subsystems.floor.FloorIOSim;   

//import static edu.wpi.first.units.Units.Degrees;
//import frc.robot.Constants;

public class RobotContainer {
    double SHOOTER_ADJUST_AMOUNT = 2.0; // amount to adjust shooter speed by when pressing the POV up/down buttons
    double TURRET_ADJUST_AMOUNT = 2.0; // amount to adjust turret angle by when pressing the POV left/right buttons

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
    private final ShooterHood shooterHood;

    // private final LedSubsystem m_led = new LedSubsystem();
    private final Field2d targetField;

    public Swerve getSwerve() {
        return s_Swerve;
    }

    /* Alliance colors */
    private final Color redBumper = Color.kDarkRed;
    private final Color blueBumper = Color.kDarkBlue;
    private Color original_color;
    public Pose2d globalRobotPose;

    public RobotContainer() {
        if (Robot.isReal()) {
            this.rack = new Rack(new RackIOReal());
            this.intake = new Intake(new IntakeIOReal());
            this.floor = new Floor(new FloorIOReal());
            this.shooter = new Shooter(new ShooterIOReal());
            this.turret = new Turret(new TurretIOReal());
            this.shooterHood = new ShooterHood(new ShooterHoodIOReal());
        } else {
            this.rack = new Rack(new RackIOSim()); // Simulated rack for testing
            this.intake = new Intake(new IntakeIOSim()); // Simulated intake for testing
            this.floor = new Floor(new FloorIOSim());
            this.shooter = new Shooter(new ShooterIOSim());
            this.turret = new Turret(new TurretIOSim());
            this.shooterHood = new ShooterHood(new ShooterHoodIOSim());
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

        turret.setDefaultCommand(Commands.run(() -> turret.setTurretAngle(getTurretAngle()), turret));
        shooterHood.setDefaultCommand((Commands.run(() -> shooterHood.setShooterHoodAngle(-getHoodAngle()), shooterHood)));

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        //driverController.povUp().onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
        //driverController.povDown().onTrue(s_Swerve.resetModulesToAbsolute());
        // Turret rotation with Triggers
        /*turret.setDefaultCommand(
            Commands.run(() -> {
                double raw = turretAxis.get();
                if (Math.abs(raw) < 0.05) return;
                turret.setTurretAngle(raw * 135.0);
            }, turret)
        );*/

        // up/down pov for incr/dec offset to shooter speed
        // left/right pov for incr/decr turret angle offsets
        driverController.povUp().onTrue(shooter.modifyOffsetCmd(SHOOTER_ADJUST_AMOUNT));   // increase shooter speed offset by 1
        driverController.povDown().onTrue(shooter.modifyOffsetCmd(-SHOOTER_ADJUST_AMOUNT));   // decrease shooter speed offset by 1
        driverController.povLeft().onTrue(turret.modifyOffsetCmd(-TURRET_ADJUST_AMOUNT));   // decrease turret angle offset by 1 degrees
        driverController.povRight().onTrue(turret.modifyOffsetCmd(TURRET_ADJUST_AMOUNT));   // increase turret angle offset by 1 degrees

        // Hold left bumper to auto-aim at hub, release to go back to trigger control
        // driverController.leftBumper().whileTrue(
        //     Commands.run(() -> turret.setTurretAngle(getTurretAngle()), turret)
        // );

        // run the intake
        driverController.leftBumper().whileTrue(intake.setIntakeSpeed(0.5))
            .onFalse(intake.stopCmd());

        // rack goes out (deployed)
        driverController.leftTrigger().onTrue(rack.rackToCmd(-34000.0));
        // rack goes in (retracted)
        driverController.rightTrigger().onTrue(rack.rackToCmd(0.0));

        driverController.a().onTrue(shooterHood.setSpeedCmd(0.1).until(shooterHood::IsOverloaded));
        // turret testing
        //driverController.b().onTrue(shooterHood.shooterHoodToCmd(-getHoodAngle())); //Negative for inverse rotation

        //driverController.x().onTrue(turret.turretToCmd(180.0));
        //driverController.x().whileTrue(shooterHood.setSpeedCmd(0.3)).onFalse(shooterHood.setSpeedCmd(0));
        //driverController.y().whileTrue(shooterHood.setSpeedCmd(-0.3)).onFalse(shooterHood.setSpeedCmd(0));
        //driverController.y().onTrue(turret.turretTo(-5.0));s
        
        driverController.rightBumper().whileTrue(shootCmd())
            .onFalse(shooter.stopCmd());
            //.alongWith(floor.setFloorSpeed(-0.25)))
            //.onFalse(shooter.stopCmd().alongWith(floor.stopCmd()));

        driverController.x().onTrue(shooter.setShooterSpeed(0));
        driverController.y().onTrue(shooterHood.shooterHoodToCmd(-100));

    }


    public Command shootCmd() {
        return shooter.continuousSetShooterSpeed(s_Swerve);
    }


    // ── Targeting ──────────────────────────────────────────────────────────────
    private double getTurretAngle() {
        Pose2d robotPose = Swerve.flipIfRed(s_Swerve.getRobotPose());
        // Translation2d target = Swerve.flipIfRed(Constants.Localization.hubPosition);
        // SmartDashboard.putString("rpose", robotPose.toString());
        // if (robotPose.getX() > Constants.Localization.trenchline) {
        //     if (robotPose.getY() < Constants.Localization.yMidline) {
        //         target = Swerve.flipIfRed(Constants.Localization.lowerPassTarget);
        //     }
        //     else {
        //         target = Swerve.flipIfRed(Constants.Localization.lowerPassTarget);
        //     }
        // }
        Translation2d target = s_Swerve.getTargetForRobotPose(robotPose);
        Translation2d toTarget = robotPose.getTranslation().minus(target);
        double targetAngle = Math.toDegrees(Math.atan2(toTarget.getY(), toTarget.getX()));
        double turretAngle = targetAngle - robotPose.getRotation().getDegrees();
        turretAngle = MathUtil.inputModulus(turretAngle + 180, -185, 185);
        SmartDashboard.putNumber("Turret Angle", turretAngle);
        SmartDashboard.putString("Target", target.toString());
        return -(MathUtil.clamp(turretAngle, -185.0, 185.0));
    }

    private double getHoodAngle() {
        double targetDistance = s_Swerve.getTargetDistance();
        double hoodAngle = targetDistance * 60; //TODO: Tune this!! Target distance * some formula
        SmartDashboard.putNumber("Hood Angle", hoodAngle);
        return Math.abs(MathUtil.clamp(hoodAngle, 100.0, 400.0));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected()
            .deadlineFor(Commands.run(() -> turret.setTurretAngle(getTurretAngle()), turret))
            .alongWith((Commands.run(() -> shooterHood.setShooterHoodAngle(-getHoodAngle()), shooterHood)));
    }

    public Turret getTurret() {
        return turret;
    }
}