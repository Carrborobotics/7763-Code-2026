package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import dev.doglog.DogLog;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CAN;
import frc.robot.Constants.Drive.Module;

/**
 * Our own swerve implementation (the one used at competition): four {@link SwerveModule}s, a Pigeon2 and
 * a WPILib pose estimator updated at the 50 Hz main loop.
 */
public final class CustomSwerve extends SubsystemBase implements SwerveDrive {
    private static final SwerveDriveKinematics KINEMATICS = Constants.Drive.KINEMATICS;

    private final SwerveModule[] modules = new SwerveModule[4];
    private final Pigeon2 gyro = new Pigeon2(CAN.PIGEON, CAN.DRIVETRAIN);
    private final StatusSignal<Angle> gyroYaw = gyro.getYaw();
    private final BaseStatusSignal[] allSignals;
    private final SwerveDrivePoseEstimator estimator;
    private final Field2d field = new Field2d();

    // Simulation has no physics for the modules, so integrate the commanded speeds instead.
    private ChassisSpeeds lastCommanded = new ChassisSpeeds();
    private Pose2d simPose = new Pose2d();

    public CustomSwerve() {
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);
        Health.register("Pigeon (" + CAN.PIGEON + ")", gyro);

        List<BaseStatusSignal> signals = new ArrayList<>(List.of(gyroYaw));
        Module[] ids = Constants.Drive.MODULES;
        for (int i = 0; i < 4; i++) {
            modules[i] = new SwerveModule(ids[i]);
            signals.addAll(List.of(modules[i].signals()));
        }
        allSignals = signals.toArray(BaseStatusSignal[]::new);
        BaseStatusSignal.setUpdateFrequencyForAll(100, allSignals);

        estimator = new SwerveDrivePoseEstimator(KINEMATICS, gyroRotation(), modulePositions(), new Pose2d(),
                VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
                VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));

        SmartDashboard.putData("Field", field);
        SwerveDrive.configurePathPlanner(this);
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(allSignals);
        estimator.update(gyroRotation(), modulePositions());

        if (RobotBase.isSimulation()) {
            simPose = simPose.exp(new Twist2d(
                    lastCommanded.vxMetersPerSecond * Constants.LOOP_PERIOD_S,
                    lastCommanded.vyMetersPerSecond * Constants.LOOP_PERIOD_S,
                    lastCommanded.omegaRadiansPerSecond * Constants.LOOP_PERIOD_S));
        }

        Pose2d pose = getPose();
        field.setRobotPose(pose);
        DogLog.log("Drive/Pose", pose);
        DogLog.log("Drive/GyroYawDeg", gyroYaw.getValueAsDouble());
        DogLog.log("Drive/MeasuredStates", moduleStates());
        DogLog.log("Drive/CommandedSpeeds", lastCommanded);
        for (SwerveModule m : modules) {
            DogLog.log("Drive/CANcoderDeg/" + m.name, m.cancoderDegrees());
        }
    }

    // ---- driving ----

    @Override
    public void driveFieldRelative(double vxMps, double vyMps, double omegaRadPerS) {
        drive(ChassisSpeeds.fromFieldRelativeSpeeds(vxMps, vyMps, omegaRadPerS, getPose().getRotation()), true);
    }

    @Override
    public void driveRobotRelative(ChassisSpeeds speeds) {
        drive(speeds, false);
    }

    /** Closed-loop velocity for autos, open-loop duty cycle for teleop. */
    private void drive(ChassisSpeeds speeds, boolean openLoop) {
        lastCommanded = speeds;
        SwerveModuleState[] states = KINEMATICS.toSwerveModuleStates(
                ChassisSpeeds.discretize(speeds, Constants.LOOP_PERIOD_S));
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Drive.MAX_SPEED_MPS);
        for (int i = 0; i < 4; i++) {
            modules[i].apply(states[i], openLoop);
        }
    }

    // ---- state ----

    @Override
    public Pose2d getPose() {
        return RobotBase.isSimulation() ? simPose : estimator.getEstimatedPosition();
    }

    @Override
    public void setPose(Pose2d pose) {
        estimator.resetPosition(gyroRotation(), modulePositions(), pose);
        simPose = pose;
    }

    @Override
    public ChassisSpeeds getRobotRelativeSpeeds() {
        return RobotBase.isSimulation() ? lastCommanded : KINEMATICS.toChassisSpeeds(moduleStates());
    }

    @Override
    public Rotation3d getTilt() {
        return RobotBase.isSimulation() ? new Rotation3d() : gyro.getRotation3d();
    }

    @Override
    public void addVisionMeasurement(Pose2d pose, double timestampS, Matrix<N3, N1> stdDevs) {
        estimator.addVisionMeasurement(pose, timestampS, stdDevs);
    }

    @Override
    public void setSysIdVolts(double volts) {
        lastCommanded = new ChassisSpeeds();
        for (SwerveModule m : modules) m.applyVoltage(volts);
    }

    @Override
    public double sysIdPositionM() {
        double sum = 0;
        for (SwerveModulePosition p : modulePositions()) sum += p.distanceMeters;
        return sum / 4;
    }

    @Override
    public double sysIdVelocityMps() {
        double sum = 0;
        for (SwerveModuleState s : moduleStates()) sum += s.speedMetersPerSecond;
        return sum / 4;
    }

    private Rotation2d gyroRotation() {
        return Rotation2d.fromDegrees(gyroYaw.getValueAsDouble());
    }

    private SwerveModulePosition[] modulePositions() {
        var positions = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) positions[i] = modules[i].position();
        return positions;
    }

    private SwerveModuleState[] moduleStates() {
        var states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) states[i] = modules[i].state();
        return states;
    }
}
