package frc.robot;

//import static edu.wpi.first.units.Units.Degrees;

//import java.util.EnumMap;
//import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.floor.Floor;
import frc.robot.subsystems.floor.FloorIOReal;
import frc.robot.subsystems.floor.FloorIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOReal;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.rack.Rack;
import frc.robot.subsystems.rack.RackIOReal;
import frc.robot.subsystems.rack.RackIOSim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIOReal;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooterhood.ShooterHood;
import frc.robot.subsystems.shooterhood.ShooterHoodIOReal;
import frc.robot.subsystems.shooterhood.ShooterHoodIOSim;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretIOReal;
import frc.robot.subsystems.turret.TurretIOSim;   
import frc.robot.util.ShooterCalc;


// hood reset debounce
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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
    //private final Supplier<Double> turretAxis = () ->
    //    driverController.getRightTriggerAxis() - driverController.getLeftTriggerAxis();

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

    // call utility class for calculating shooter parameters based on robot pose
    // set targeting mode to one of the 3 modes: BASIC, SOTM, SOTM_CONVERGED
    private final ShooterCalc shootcalc = new ShooterCalc(s_Swerve, ShooterCalc.TargetingMode.SOTM);

    public Swerve getSwerve() {
        return s_Swerve;
    }

    /* Alliance colors */
    // private final Color redBumper = Color.kDarkRed;
    // private final Color blueBumper = Color.kDarkBlue;
    // private Color original_color;
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

        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                    s_Swerve,
                    () -> -translationAxis.get(),
                    () -> -strafeAxis.get(),
                    () -> -rotationAxis.get()
                    // () -> false,
                    // () -> false
            )
        );

        NamedCommands.registerCommand("Shoot_5s", shootCmd().withTimeout(5.0));
        NamedCommands.registerCommand("Shoot_Off", shooter.stopCmd());
        NamedCommands.registerCommand("Intake_On", intake.setIntakeSpeed(0.5));
        NamedCommands.registerCommand("Intake_Off", intake.setIntakeSpeed(0));
        NamedCommands.registerCommand("Rack_Extend", rack.rackToCmd(Constants.RACK_EXTEND_POSITION));
        NamedCommands.registerCommand("Rack_Retract", rack.rackToCmd(Constants.RACK_RETRACT_POSITION));
        NamedCommands.registerCommand("Rack_80pct", rack.rackToCmd(Constants.RACK_EXTEND_POSITION*0.8));
        NamedCommands.registerCommand("Rack_70pct", rack.rackToCmd(Constants.RACK_EXTEND_POSITION*0.7));
        NamedCommands.registerCommand("Rack_60pct", rack.rackToCmd(Constants.RACK_EXTEND_POSITION*0.6));
        NamedCommands.registerCommand("Rack_50pct", rack.rackToCmd(Constants.RACK_EXTEND_POSITION*0.5));
        NamedCommands.registerCommand("Agitate", agitateCommand());
        
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        turret.setDefaultCommand(Commands.run(() -> turret.setTurretAngle(shootcalc.getTurretAngle()), turret));
        shooterHood.setDefaultCommand((Commands.run(() -> shooterHood.setShooterHoodAngle(-shootcalc.getHoodAngle()), shooterHood)));

        configureButtonBindings();
    }

    private void configureButtonBindings() {
    
        // up/down pov for incr/dec offset to shooter speed
        // left/right pov for incr/decr turret angle offsets
        driverController.povUp().onTrue(shooter.modifyOffsetCmd(SHOOTER_ADJUST_AMOUNT));   // increase shooter speed offset by 1
        driverController.povDown().onTrue(shooter.modifyOffsetCmd(-SHOOTER_ADJUST_AMOUNT));   // decrease shooter speed offset by 1
        driverController.povLeft().onTrue(turret.modifyOffsetCmd(-TURRET_ADJUST_AMOUNT));   // decrease turret angle offset by 1 degrees
        driverController.povRight().onTrue(turret.modifyOffsetCmd(TURRET_ADJUST_AMOUNT));   // increase turret angle offset by 1 degrees

        // run the intake
        driverController.leftBumper().whileTrue(intake.setIntakeSpeed(Constants.Intake.FORWARD_SPEED))
            .onFalse(intake.stopCmd());

        // rack goes out (deployed)
        driverController.leftTrigger().onTrue(rack.rackToCmd(Constants.RACK_EXTEND_POSITION));
        // rack goes in (retracted)
        driverController.rightTrigger().onTrue(rack.rackToCmd(Constants.RACK_RETRACT_POSITION));

        // attempt to reset hood
        Trigger hoodLimitSensed = new Trigger(() -> shooterHood.IsOverloaded()).debounce(0.75, DebounceType.kBoth);
        driverController.a().onTrue(shooterHood.setSpeedCmd(0.5).until(hoodLimitSensed));
        
        driverController.rightBumper().whileTrue(shootCmd())
            .onFalse(shooter.stopCmd().alongWith(floor.stopCmd()));
            //.alongWith(floor.setFloorSpeed(-0.25)))
            //.onFalse(shooter.stopCmd().alongWith(floor.stopCmd()));

        //driverController.x().onTrue(shooter.setShooterSpeed(0));
        //driverController.y().onTrue(shooterHood.shooterHoodToCmd(-100));

        // partial rack position
        driverController.x().onTrue(rack.rackToCmd(Constants.RACK_EXTEND_POSITION*0.50));
        driverController.y().onTrue(agitateCommand());

        // testing indexer(floor) speeds
        //driverController.start().whileTrue(floor.setFloorSpeed(-0.5)).onFalse(floor.stopCmd()); // turbo floor speed
        driverController.start().whileTrue(intake.setIntakeSpeed(-0.2)).onFalse(intake.stopCmd()); // reverse intake
        driverController.back().whileTrue(floor.setFloorSpeed(-0.2)).onFalse(floor.stopCmd()); // normal floor speed
        driverController.b().whileTrue(floor.setFloorSpeed(-0.2).alongWith(shooter.setShooterSpeed(0.2)))
            .onFalse(floor.stopCmd().alongWith(shooter.stopCmd())); // reverse floor speed (ejecting balls)
        
    }

    public Command shootCmd() {
        return shooter.continuousSetShooterSpeed(shootcalc).alongWith(floor.setFloorSpeed(0.35));
    }
    public Command agitateCommand() {
        return new SequentialCommandGroup(
            rack.rackToCmd(Constants.RACK_EXTEND_POSITION*1.00),
            new WaitCommand(0.5),
            rack.rackToCmd(Constants.RACK_EXTEND_POSITION*0.50),
            new WaitCommand(0.5),
            rack.rackToCmd(Constants.RACK_EXTEND_POSITION*0.75),
            new WaitCommand(0.5),
            rack.rackToCmd(Constants.RACK_EXTEND_POSITION*0.50),
            new WaitCommand(0.5),
            rack.rackToCmd(Constants.RACK_EXTEND_POSITION*1.00)
        );
    }

    public Command shootAndSqueezeCommand() {
        return shooter.setShooterSpeed(-0.5).withTimeout(1).andThen(
            shooter.continuousSetShooterSpeed(shootcalc)
                .alongWith(floor.setFloorSpeed(-0.6))
                .alongWith(
                    rack.rackToCmd(Constants.RACK_EXTEND_POSITION*0.9)
                )
        );
    }


    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected()
            .deadlineFor(Commands.run(() -> turret.setTurretAngle(shootcalc.getTurretAngle()), turret))
            .alongWith((Commands.run(() -> shooterHood.setShooterHoodAngle(-shootcalc.getHoodAngle()), shooterHood)));
    }

    public Turret getTurret() {
        return turret;
    }
}