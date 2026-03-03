package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurretSubsystem;

/**
 * ShootCommand
 *
 * Spins the flywheel and sets the hood/turret to a desired state,
 * then waits until all three are on target before signaling isFinished.
 *
 * Usage example (in RobotContainer):
 *
 *   new JoystickButton(controller, XboxController.Button.kRightBumper.value)
 *       .whileTrue(new ShootCommand(turret, 3000, 45.0, 0.0));
 */
public class ShootCommand extends Command {

    private final TurretSubsystem turret;
    private final double targetRPM;
    private final double hoodAngleDegrees;
    private final double turretAngleDegrees;

    /**
     * @param turret              The TurretSubsystem to use
     * @param targetRPM           Desired flywheel RPM
     * @param hoodAngleDegrees    Desired hood angle in degrees
     * @param turretAngleDegrees  Desired turret angle in degrees (0 = forward)
     */
    public ShootCommand(TurretSubsystem turret, double targetRPM,
                        double hoodAngleDegrees, double turretAngleDegrees) {
        this.turret = turret;
        this.targetRPM = targetRPM;
        this.hoodAngleDegrees = hoodAngleDegrees;
        this.turretAngleDegrees = turretAngleDegrees;
        addRequirements(turret);
    }

    @Override
    public void initialize() {
        turret.setShooterRPM(targetRPM);
        turret.setHoodAngle(hoodAngleDegrees);
        turret.setTurretAngle(turretAngleDegrees);
    }

    @Override
    public void execute() {
        // Setpoints are already applied; TalonFX closed-loop handles the rest.
        // Add indexer/feeder logic here to actually feed the ball once ready:
        //   if (turret.isReadyToShoot()) { indexer.run(); }
    }

    @Override
    public boolean isFinished() {
        // Finish once everything is on target.
        // If used with whileTrue(), isFinished() is ignored and end() handles cleanup.
        return turret.isReadyToShoot();
    }

    @Override
    public void end(boolean interrupted) {
        turret.stopShooter();
        // Optionally return hood to stow angle:
        turret.setHoodAngle(20.0);
        // Leave turret at last angle or return to center:
        // turret.setTurretAngle(0.0);
    }
}