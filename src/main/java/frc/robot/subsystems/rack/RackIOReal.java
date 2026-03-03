package frc.robot.subsystems.rack;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import static edu.wpi.first.units.Units.*;

import frc.robot.Constants;

public class RackIOReal implements RackIO {

    private final TalonFX rackMotorLeft;
    private final TalonFX rackMotorRight;

    private final MotionMagicVoltage m_mmRequest = new MotionMagicVoltage(0)
            .withSlot(0)
            .withEnableFOC(false); // set true once FOC license is setup

    public RackIOReal() {
        rackMotorLeft  = new TalonFX(Constants.CANConstants.rackId,  Constants.CANConstants.canBus);
        rackMotorRight = new TalonFX(Constants.CANConstants.rackId2, Constants.CANConstants.canBus);

        var cfg = new TalonFXConfiguration();

        // --- Neutral mode ---
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // --- Current limits ---
        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        cfg.CurrentLimits.SupplyCurrentLimit       = 10;
        cfg.CurrentLimits.SupplyCurrentLowerLimit  = 8;
        cfg.CurrentLimits.SupplyCurrentLowerTime   = 0.5;

        // --- Voltage limits ---
        cfg.Voltage.PeakForwardVoltage = 12;
        cfg.Voltage.PeakReverseVoltage = 12;

        // --- Soft limits ---
        cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable    = true;
        cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 10.5; // TODO: tune
        cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable    = true;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -0.5; // TODO: tune

        // --- Slot 0 PID gains ---
        cfg.Slot0.kP = 2.4;  // TODO: tune
        cfg.Slot0.kI = 0.0;
        cfg.Slot0.kD = 0.1;  // TODO: tune
        cfg.Slot0.kS = 0.25; // TODO: tune
        cfg.Slot0.kV = 0.12; // TODO: tune
        cfg.Slot0.kA = 0.01; // TODO: tune

        // --- MotionMagic ---
        cfg.MotionMagic.MotionMagicCruiseVelocity = 80;   // RPS   TODO: tune
        cfg.MotionMagic.MotionMagicAcceleration   = 160;  // RPS/s TODO: tune
        cfg.MotionMagic.MotionMagicJerk            = 1600; // RPS/s²

        // Apply config to leader only
        rackMotorLeft.getConfigurator().apply(cfg);

        // Right motor follows left, opposed because physically mirrored
        rackMotorRight.setControl(new Follower(rackMotorLeft.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    @Override
    public void updateInputs(RackIOInputs inputs) {
        inputs.appliedVolts.mut_replace(rackMotorLeft.getSupplyVoltage().getValueAsDouble(), Volts);
        inputs.supplyCurrent.mut_replace(rackMotorLeft.getSupplyCurrent().getValueAsDouble(), Amps);
        inputs.torqueCurrent.mut_replace(rackMotorLeft.getTorqueCurrent().getValueAsDouble(), Amps);
        inputs.temperature.mut_replace(rackMotorLeft.getDeviceTemp().getValueAsDouble(), Celsius);
        inputs.velocity.mut_replace(rackMotorLeft.getVelocity().getValueAsDouble(), DegreesPerSecond);
    }

    @Override
    public void setPosition(double rotations) {
        rackMotorLeft.setControl(m_mmRequest.withPosition(rotations));
    }

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