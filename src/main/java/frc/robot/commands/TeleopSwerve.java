package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class TeleopSwerve extends Command {
    private Swerve s_Swerve;
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier sotmSup;
    // private BooleanSupplier useCoralLimelightSup;

    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup,
            DoubleSupplier rotationSup,
            BooleanSupplier sotmSup
            //, BooleanSupplier sotmSup, BooleanSupplier useCoralLimelightSup
            ) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.sotmSup = sotmSup;
        // this.useCoralLimelightSup = useCoralLimelightSup;

    }

    @Override
    public void execute() {
        /* Get Values, Deadband */
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband) / Constants.stickDeadbandDivider;
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband) / Constants.stickDeadbandDivider;
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband) / Constants.stickDeadbandDivider;

        // Driver position is inverted for Red alliance, so adjust field-oriented controls
        if (Robot.isRed()) {
            translationVal *= -1.0;
            strafeVal *= -1.0;
        }

        double maxspeed = Constants.Swerve.maxSpeed;
        double maxAngularVelocity = Constants.Swerve.maxAngularVelocity;
        if (sotmSup.getAsBoolean()) {
            maxspeed = Constants.Swerve.maxSpeedSOTM;
            maxAngularVelocity = Constants.Swerve.maxAngularVelocitySOTM;
        }
        SmartDashboard.putNumber("Swerve/maxSpeed", maxspeed);
        /* Drive */
        s_Swerve.drive(
                0.0,
                0.0,
                new Translation2d(translationVal, strafeVal).times(maxspeed),
                rotationVal * maxAngularVelocity,
                true,
                false,
                false //useCoralLimelightSup.getAsBoolean()
            );
    }
}
