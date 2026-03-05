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
    private final Turret turret = new Turret();
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
        } else {
            this.rack = new Rack(new RackIOSim());
            this.intake = new Intake(new IntakeIOSim());
        }

        NamedCommands.registerCommand("Aim Hub", Commands.run(() -> turret.setTurretAngle(getAngleToHub()), turret));

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

        driverController.a().onTrue(new InstantCommand(() -> rack.setPosition(-5.0)));
        driverController.b().onTrue(new InstantCommand(() -> rack.setPosition(0.0)));
        driverController.x().onTrue(new InstantCommand(() -> rack.setPosition(5.0)));

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