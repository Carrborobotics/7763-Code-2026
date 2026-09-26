package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants.CAN;
import frc.robot.Constants.Drive;

/** One swerve module: Kraken drive, Kraken steer, CANcoder. Units are wheel rotations. */
final class SwerveModule {
    final String name;
    private final TalonFX drive;
    private final TalonFX steer;
    private final CANcoder encoder;
    private final Rotation2d offset;

    private final StatusSignal<Angle> drivePos;
    private final StatusSignal<AngularVelocity> driveVel;
    private final StatusSignal<Angle> steerPos;
    private final StatusSignal<AngularVelocity> steerVel;

    private final SimpleMotorFeedforward driveFF =
            new SimpleMotorFeedforward(Drive.DRIVE_KS, Drive.DRIVE_KV, Drive.DRIVE_KA);
    private final DutyCycleOut driveDuty = new DutyCycleOut(0).withEnableFOC(true);
    private final VelocityVoltage driveVelocity = new VelocityVoltage(0).withEnableFOC(true);
    private final PositionVoltage steerPosition = new PositionVoltage(0).withEnableFOC(true);
    private final VoltageOut driveVoltage = new VoltageOut(0).withEnableFOC(true);

    SwerveModule(Drive.Module ids) {
        name = ids.name();
        offset = ids.offset();

        var encoderCfg = new CANcoderConfiguration();
        encoderCfg.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        encoder = new CANcoder(ids.cancoderId(), CAN.DRIVETRAIN);
        encoder.getConfigurator().apply(encoderCfg);
        Health.register(name + " CANcoder (" + ids.cancoderId() + ")", encoder);

        var steerCfg = Talons.config(InvertedValue.Clockwise_Positive, NeutralModeValue.Coast);
        steerCfg.Feedback.SensorToMechanismRatio = Drive.STEER_RATIO;
        steerCfg.ClosedLoopGeneral.ContinuousWrap = true;
        steerCfg.Slot0.kP = Drive.STEER_KP;
        steer = Talons.create(name + " steer", ids.steerId(), CAN.DRIVETRAIN, steerCfg);

        var driveCfg = Talons.config(InvertedValue.CounterClockwise_Positive, NeutralModeValue.Brake);
        driveCfg.Feedback.SensorToMechanismRatio = Drive.DRIVE_RATIO;
        driveCfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        driveCfg.CurrentLimits.SupplyCurrentLimit = Drive.DRIVE_SUPPLY_LIMIT_A;
        driveCfg.CurrentLimits.SupplyCurrentLowerLimit = Drive.DRIVE_SUPPLY_LIMIT_A;
        driveCfg.CurrentLimits.SupplyCurrentLowerTime = 0.1;
        driveCfg.Slot0.kP = Drive.DRIVE_KP;
        driveCfg.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Drive.OPEN_LOOP_RAMP_S;
        driveCfg.OpenLoopRamps.VoltageOpenLoopRampPeriod = Drive.OPEN_LOOP_RAMP_S;
        drive = Talons.create(name + " drive", ids.driveId(), CAN.DRIVETRAIN, driveCfg);
        drive.setPosition(0);

        drivePos = drive.getPosition();
        driveVel = drive.getVelocity();
        steerPos = steer.getPosition();
        steerVel = steer.getVelocity();

        seedSteerFromCancoder();
    }

    /** Point the steer encoder at the true wheel angle. Waits for a fresh CANcoder reading first. */
    void seedSteerFromCancoder() {
        double absolute = encoder.getAbsolutePosition().waitForUpdate(0.25).getValueAsDouble();
        steer.setPosition(absolute - offset.getRotations());
    }

    /** Signals Drive refreshes together once per loop. */
    BaseStatusSignal[] signals() {
        return new BaseStatusSignal[] {drivePos, driveVel, steerPos, steerVel};
    }

    private Rotation2d angle() {
        return Rotation2d.fromRotations(BaseStatusSignal.getLatencyCompensatedValueAsDouble(steerPos, steerVel));
    }

    SwerveModulePosition position() {
        double wheelRot = BaseStatusSignal.getLatencyCompensatedValueAsDouble(drivePos, driveVel);
        return new SwerveModulePosition(wheelRot * Drive.WHEEL_CIRCUMFERENCE_M, angle());
    }

    SwerveModuleState state() {
        return new SwerveModuleState(driveVel.getValueAsDouble() * Drive.WHEEL_CIRCUMFERENCE_M, angle());
    }

    double cancoderDegrees() {
        return encoder.getAbsolutePosition().getValueAsDouble() * 360.0;
    }

    /** SysId: wheel straight ahead, fixed drive voltage. */
    void applyVoltage(double volts) {
        steer.setControl(steerPosition.withPosition(0));
        drive.setControl(driveVoltage.withOutput(volts));
    }

    void apply(SwerveModuleState desired, boolean openLoop) {
        Rotation2d current = angle();
        desired.optimize(current);
        desired.cosineScale(current); // don't drive hard while the wheel is still turning into place
        steer.setControl(steerPosition.withPosition(desired.angle.getRotations()));

        double mps = desired.speedMetersPerSecond;
        if (openLoop) {
            drive.setControl(driveDuty.withOutput(mps / Drive.MAX_SPEED_MPS));
        } else {
            drive.setControl(driveVelocity
                    .withVelocity(mps / Drive.WHEEL_CIRCUMFERENCE_M)
                    .withFeedForward(driveFF.calculate(mps)));
        }
    }
}
