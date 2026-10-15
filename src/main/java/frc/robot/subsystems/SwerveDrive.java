package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Robot;

/**
 * What the rest of the robot needs from a drivetrain. Two implementations: {@link CustomSwerve} (our own
 * module code) and {@link CtreSwerve} (CTRE's swerve library); Constants.Drive.USE_CTRE_SWERVE picks one.
 * Poses are always blue-origin field coordinates.
 */
public interface SwerveDrive extends Subsystem {
    Pose2d getPose();

    void setPose(Pose2d pose);

    /** Keep position, change heading. Used by the driver's "reset heading" button. */
    default void resetHeading(Rotation2d heading) {
        setPose(new Pose2d(getPose().getTranslation(), heading));
    }

    /** Teleop: field-relative (blue-origin frame), open loop. */
    void driveFieldRelative(double vxMps, double vyMps, double omegaRadPerS);

    /** Autos: robot-relative, closed-loop velocity. */
    void driveRobotRelative(ChassisSpeeds speeds);

    ChassisSpeeds getRobotRelativeSpeeds();

    /** Robot velocity in the blue-origin field frame (x, y), m/s. */
    default Translation2d getFieldVelocity() {
        ChassisSpeeds s = getRobotRelativeSpeeds();
        return new Translation2d(s.vxMetersPerSecond, s.vyMetersPerSecond).rotateBy(getPose().getRotation());
    }

    /** Full chassis orientation (roll, pitch, yaw). */
    Rotation3d getTilt();

    void addVisionMeasurement(Pose2d pose, double timestampS, Matrix<N3, N1> stdDevs);

    // ---- SysId (test mode) ----

    /** Point every wheel forward and apply this voltage to every drive motor. */
    void setSysIdVolts(double volts);

    /** Average drive-wheel distance (m) and speed (m/s), for SysId logs. */
    double sysIdPositionM();

    double sysIdVelocityMps();

    /** Wheel angles relative to the robot (deg, CCW+, 0 = forward) in FL, FR, BL, BR order, for the match view. */
    static double[] moduleAnglesDeg(SwerveModuleState[] states) {
        double[] deg = new double[states.length];
        for (int i = 0; i < states.length; i++) deg[i] = states[i].angle.getDegrees();
        return deg;
    }

    /** Hook up PathPlanner. Both implementations call this once from their constructor. */
    static void configurePathPlanner(SwerveDrive drive) {
        try {
            AutoBuilder.configure(
                    drive::getPose,
                    drive::setPose,
                    drive::getRobotRelativeSpeeds,
                    (speeds, feedforwards) -> drive.driveRobotRelative(speeds),
                    new PPHolonomicDriveController(
                            new PIDConstants(Constants.Drive.AUTO_TRANSLATION_KP),
                            new PIDConstants(Constants.Drive.AUTO_ROTATION_KP)),
                    RobotConfig.fromGUISettings(),
                    Robot::isRed,
                    drive);
        } catch (Exception e) {
            DriverStation.reportError("PathPlanner config failed; autos will not run: " + e.getMessage(), e.getStackTrace());
        }
    }
}
