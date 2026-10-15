package frc.robot.subsystems;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import dev.doglog.DogLog;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CAN;
import frc.robot.Constants.Drive;

/**
 * CTRE's swerve library, configured from the same numbers as {@link CustomSwerve}: odometry runs on its
 * own 250 Hz thread with time-synced CAN FD signals. Built by hand from Constants instead of Tuner X
 * so there is exactly one copy of every ID, offset and ratio.
 *
 * <p>Conversions from our conventions: CTRE adds the encoder offset (ours was subtracted), steer
 * "inverted" = clockwise-positive (matches our steer motors), drive motors not inverted, and drive
 * gains are per rotor rotation (see Constants.Drive.CTRE_*).
 */
public final class CtreSwerve extends SubsystemBase implements SwerveDrive {
    private final SwerveDrivetrain<TalonFX, TalonFX, CANcoder> drivetrain;
    private final Field2d field = new Field2d();

    private final SwerveRequest.FieldCentric fieldCentric = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);
    private final SwerveRequest.ApplyRobotSpeeds robotSpeeds = new SwerveRequest.ApplyRobotSpeeds()
            .withDriveRequestType(DriveRequestType.Velocity);
    private final SwerveRequest.SysIdSwerveTranslation sysId = new SwerveRequest.SysIdSwerveTranslation();

    public CtreSwerve() {
        var driveInitial = new TalonFXConfiguration();
        driveInitial.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveInitial.CurrentLimits.SupplyCurrentLimitEnable = true;
        driveInitial.CurrentLimits.SupplyCurrentLimit = Drive.DRIVE_SUPPLY_LIMIT_A;
        driveInitial.CurrentLimits.SupplyCurrentLowerLimit = Drive.DRIVE_SUPPLY_LIMIT_A;
        driveInitial.CurrentLimits.SupplyCurrentLowerTime = 0.1;
        driveInitial.OpenLoopRamps.VoltageOpenLoopRampPeriod = Drive.OPEN_LOOP_RAMP_S;
        var steerInitial = new TalonFXConfiguration();
        steerInitial.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        var factory = new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
                .withDriveMotorGearRatio(Drive.DRIVE_RATIO)
                .withSteerMotorGearRatio(Drive.STEER_RATIO)
                .withCouplingGearRatio(0) // our own code never compensated coupling either
                .withWheelRadius(Drive.WHEEL_DIAMETER_M / 2)
                .withSteerMotorGains(new Slot0Configs().withKP(Drive.STEER_KP))
                .withDriveMotorGains(new Slot0Configs()
                        .withKP(Drive.CTRE_DRIVE_KP).withKS(Drive.CTRE_DRIVE_KS)
                        .withKV(Drive.CTRE_DRIVE_KV).withKA(Drive.CTRE_DRIVE_KA))
                .withSteerMotorClosedLoopOutput(ClosedLoopOutputType.Voltage)
                .withDriveMotorClosedLoopOutput(ClosedLoopOutputType.Voltage)
                .withSlipCurrent(120)
                .withSpeedAt12Volts(Drive.MAX_SPEED_MPS) // same stick-to-voltage mapping as our duty-cycle teleop
                .withDriveMotorType(DriveMotorArrangement.TalonFX_Integrated)
                .withSteerMotorType(SteerMotorArrangement.TalonFX_Integrated)
                .withFeedbackSource(SteerFeedbackType.RemoteCANcoder) // works with or without Phoenix Pro
                .withDriveMotorInitialConfigs(driveInitial)
                .withSteerMotorInitialConfigs(steerInitial)
                .withEncoderInitialConfigs(new CANcoderConfiguration())
                .withSteerInertia(0.01)
                .withDriveInertia(0.01)
                .withSteerFrictionVoltage(0.2)
                .withDriveFrictionVoltage(0.2);

        double x = Drive.WHEEL_BASE_M / 2, y = Drive.TRACK_WIDTH_M / 2;
        double[][] locations = {{x, y}, {x, -y}, {-x, y}, {-x, -y}}; // FL, FR, BL, BR (matches Drive.MODULES)
        SwerveModuleConstants<?, ?, ?>[] modules = new SwerveModuleConstants<?, ?, ?>[4];
        for (int i = 0; i < 4; i++) {
            Drive.Module m = Drive.MODULES[i];
            modules[i] = factory.createModuleConstants(m.steerId(), m.driveId(), m.cancoderId(),
                    -m.offset().getRotations(), locations[i][0], locations[i][1],
                    false, // drive motor: counter-clockwise positive
                    true, // steer motor: clockwise positive
                    false); // CANcoder: counter-clockwise positive
        }

        drivetrain = new SwerveDrivetrain<>(TalonFX::new, TalonFX::new, CANcoder::new,
                new SwerveDrivetrainConstants().withCANBusName(CAN.DRIVETRAIN.getName()).withPigeon2Id(CAN.PIGEON),
                Drive.ODOMETRY_HZ,
                VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
                VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)),
                modules);

        for (int i = 0; i < 4; i++) {
            var module = drivetrain.getModule(i);
            String name = Drive.MODULES[i].name();
            Health.register(name + " drive (" + Drive.MODULES[i].driveId() + ")", module.getDriveMotor());
            Health.register(name + " steer (" + Drive.MODULES[i].steerId() + ")", module.getSteerMotor());
            MotorTelemetry.register(name + " drive", Drive.MODULES[i].driveId(), CAN.DRIVETRAIN, module.getDriveMotor(), driveInitial);
            MotorTelemetry.register(name + " steer", Drive.MODULES[i].steerId(), CAN.DRIVETRAIN, module.getSteerMotor(), steerInitial);
            Health.register(name + " CANcoder (" + Drive.MODULES[i].cancoderId() + ")", module.getEncoder());
        }
        Health.register("Pigeon (" + CAN.PIGEON + ")", drivetrain.getPigeon2());

        SmartDashboard.putData("Field", field);
        SwerveDrive.configurePathPlanner(this);
    }

    @Override
    public void periodic() {
        var state = drivetrain.getState();
        field.setRobotPose(state.Pose);
        DogLog.log("Drive/Pose", state.Pose);
        DogLog.log("Drive/MeasuredStates", state.ModuleStates);
        DogLog.log("Drive/ModuleAngles", SwerveDrive.moduleAnglesDeg(state.ModuleStates));
        DogLog.log("Drive/TargetStates", state.ModuleTargets);
        DogLog.log("Drive/OdometryHz", 1.0 / Math.max(1e-3, state.OdometryPeriod));
        DogLog.log("Drive/FailedDaqs", state.FailedDaqs);
    }

    @Override
    public void simulationPeriodic() {
        drivetrain.updateSimState(Constants.LOOP_PERIOD_S, RobotController.getBatteryVoltage());
    }

    @Override
    public Pose2d getPose() {
        return drivetrain.getState().Pose;
    }

    @Override
    public void setPose(Pose2d pose) {
        drivetrain.resetPose(pose);
    }

    @Override
    public void driveFieldRelative(double vxMps, double vyMps, double omegaRadPerS) {
        drivetrain.setControl(fieldCentric.withVelocityX(vxMps).withVelocityY(vyMps).withRotationalRate(omegaRadPerS));
    }

    @Override
    public void driveRobotRelative(ChassisSpeeds speeds) {
        drivetrain.setControl(robotSpeeds.withSpeeds(speeds));
    }

    @Override
    public ChassisSpeeds getRobotRelativeSpeeds() {
        return drivetrain.getState().Speeds;
    }

    @Override
    public Rotation3d getTilt() {
        return drivetrain.getRotation3d();
    }

    @Override
    public void addVisionMeasurement(Pose2d pose, double timestampS, Matrix<N3, N1> stdDevs) {
        // CTRE's estimator runs on its own clock; PhotonVision timestamps are FPGA time.
        drivetrain.addVisionMeasurement(pose, Utils.fpgaToCurrentTime(timestampS), stdDevs);
    }

    @Override
    public void setSysIdVolts(double volts) {
        drivetrain.setControl(sysId.withVolts(volts));
    }

    @Override
    public double sysIdPositionM() {
        double sum = 0;
        for (SwerveModulePosition p : drivetrain.getState().ModulePositions) sum += p.distanceMeters;
        return sum / 4;
    }

    @Override
    public double sysIdVelocityMps() {
        double sum = 0;
        for (SwerveModuleState s : drivetrain.getState().ModuleStates) sum += s.speedMetersPerSecond;
        return sum / 4;
    }
}
