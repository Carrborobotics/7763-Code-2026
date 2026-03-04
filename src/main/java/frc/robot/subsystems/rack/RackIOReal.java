package frc.robot.subsystems.rack;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.Distance;

import static edu.wpi.first.units.Units.*;

import frc.robot.Constants;

public class RackIOReal implements RackIO {

    private final TalonFX rackMotorLeft;
    private final TalonFX rackMotorRight;
    private final PositionVoltage positionRequest = new PositionVoltage(0);//.withEnableFOC(true);

    // private final MotionMagicVoltage m_mmRequest = new MotionMagicVoltage(0)
    //         .withSlot(0)
    //         .withEnableFOC(false); // set true once FOC license is setup

    public RackIOReal() {
        rackMotorLeft  = new TalonFX(Constants.CANConstants.rackId,  Constants.CANConstants.canBus);
        rackMotorRight = new TalonFX(Constants.CANConstants.rackId2, Constants.CANConstants.canBus);

         var config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        // Basic PID from TurretConstants (only kP/kI/kD applied to Slot0)
        config.Slot0.kP = 0.8; //TurretConstants.TalonFXGains.kP();
        config.Slot0.kI = 0.0; //TurretConstants.TalonFXGains.kI();
        config.Slot0.kD = 0.0; //TurretConstants.TalonFXGains.kD();

        // Set invert and apply configuration
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        rackMotorLeft.getConfigurator().apply(config);

        // var cfg = new TalonFXConfiguration();

        // // --- Neutral mode ---
        // cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // // --- Current limits ---
        // cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        // cfg.CurrentLimits.SupplyCurrentLimit       = 10;
        // cfg.CurrentLimits.SupplyCurrentLowerLimit  = 8;
        // cfg.CurrentLimits.SupplyCurrentLowerTime   = 0.5;

        // // --- Voltage limits ---
        // cfg.Voltage.PeakForwardVoltage = 12;
        // cfg.Voltage.PeakReverseVoltage = 12;

        // // --- Soft limits ---
        // //cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable    = true;
        // //cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 10.5; // TODO: tune
        // //cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable    = true;
        // //cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -0.5; // TODO: tune

        // // --- Slot 0 PID gains ---
        // cfg.Slot0.kP = 0.1;  // TODO: tune
        // cfg.Slot0.kI = 0.0;
        // cfg.Slot0.kD = 0.0;  // TODO: tune
        // cfg.Slot0.kS = 0.0; // TODO: tune
        // cfg.Slot0.kV = 0.0; // TODO: tune
        // cfg.Slot0.kA = 0.0; // TODO: tune

        // // --- MotionMagic ---
        // cfg.MotionMagic.MotionMagicCruiseVelocity = 2;   // RPS   TODO: tune
        // cfg.MotionMagic.MotionMagicAcceleration   = 4;  // RPS/s TODO: tune
        // cfg.MotionMagic.MotionMagicJerk            = 8; // RPS/s²

        // Apply config to leader only
        rackMotorLeft.getConfigurator().apply(config);

        // Right motor follows left, opposed because physically mirrored
        rackMotorRight.setControl(new Follower(rackMotorLeft.getDeviceID(), MotorAlignmentValue.Aligned));

        //rackMotorLeft.getEncoder().setPosition(0);
        rackMotorLeft.setPosition(0);

    }

    @Override
    public void updateInputs(RackIOInputs inputs) {
        // Position/velocity from TalonFX are reported in rotations; convert to radians
        double positionRotations = rackMotorLeft.getPosition().getValueAsDouble();
        double velocityRps = rackMotorLeft.getVelocity().getValueAsDouble();

        //inputs.position.mut_replace(edu.wpi.first.math.util.Units.rotationsToRadians(positionRotations), Radians);
        inputs.position.mut_replace(positionRotations, Inches);
        inputs.velocity.mut_replace(edu.wpi.first.math.util.Units.rotationsToRadians(velocityRps), RadiansPerSecond);

        inputs.appliedVolts.mut_replace(rackMotorLeft.getMotorVoltage().getValueAsDouble(), Volts);
        inputs.supplyCurrent.mut_replace(rackMotorLeft.getStatorCurrent().getValueAsDouble(), Amps);
        inputs.torqueCurrent.mut_replace(rackMotorLeft.getTorqueCurrent().getValueAsDouble(), Amps);
        inputs.temperature.mut_replace(rackMotorLeft.getDeviceTemp().getValueAsDouble(), Celsius);
    }

    @Override
    public void runSetpoint(Distance position) {
        // Convert Angle -> rotations (rotations = radians / 2pi)
        double rotations = position.in(Inches) / (2.0 * Math.PI);
        rackMotorLeft.setControl(positionRequest.withPosition(rotations));
    }

    // public void updateInputs(RackIOInputs inputs) {
    //     inputs.appliedVolts.mut_replace(rackMotorLeft.getSupplyVoltage().getValueAsDouble(), Volts);
    //     inputs.supplyCurrent.mut_replace(rackMotorLeft.getSupplyCurrent().getValueAsDouble(), Amps);
    //     inputs.torqueCurrent.mut_replace(rackMotorLeft.getTorqueCurrent().getValueAsDouble(), Amps);
    //     inputs.temperature.mut_replace(rackMotorLeft.getDeviceTemp().getValueAsDouble(), Celsius);
    //     inputs.velocity.mut_replace(rackMotorLeft.getVelocity().getValueAsDouble(), DegreesPerSecond);
    // }

    // @Override
    // public void setPosition(double rotations) {
    //     rackMotorLeft.setControl(m_mmRequest.withPosition(rotations));
    // }

    @Override
    public void setSpeed(double speed) {
        rackMotorLeft.set(speed);
    }

    @Override
    public void setVoltage(double volts) {
        rackMotorLeft.setVoltage(volts);
    }

    @Override
    public void resetPosition(double knownRotations) {
        rackMotorLeft.setPosition(knownRotations);
    }

    @Override
    public double getPosition() {
        return rackMotorLeft.getPosition().getValueAsDouble();
    }
}