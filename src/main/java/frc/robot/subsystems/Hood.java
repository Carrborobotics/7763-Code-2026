package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.math.system.plant.DCMotor;

import frc.robot.Constants;
import frc.robot.Constants.CAN;

/**
 * Shooter hood, commanded in legacy "hood units" (see Constants.Hood). The hood MUST be fully down
 * at boot, or be re-homed with the start button.
 */
public final class Hood {
    private final TalonFX motor;
    private final PositionVoltage position = new PositionVoltage(0);
    private final VoltageOut voltage = new VoltageOut(0);
    private double goalUnits = 0;

    public Hood() {
        var cfg = Talons.config(InvertedValue.CounterClockwise_Positive, NeutralModeValue.Brake);
        cfg.Slot0.kP = Constants.Hood.KP;
        cfg.Slot0.kS = Constants.Hood.KS;
        cfg.Slot0.kV = Constants.Hood.KV;
        cfg.Feedback.SensorToMechanismRatio = Constants.Hood.SENSOR_TO_MECHANISM;
        cfg.CurrentLimits.StatorCurrentLimitEnable = true;
        cfg.CurrentLimits.StatorCurrentLimit = Constants.Hood.STATOR_LIMIT_A;
        motor = Talons.create("Hood", CAN.HOOD, CAN.RIO, cfg);
        MechanismSim.add(motor, cfg, DCMotor.getKrakenX44(1), Constants.Sim.HOOD_MOI);
        motor.setPosition(0);
    }

    public void setUnits(double units) {
        goalUnits = units;
        motor.setControl(position.withPosition(-units / 360.0));
    }

    /** Open-loop drive, used only while homing. */
    public void setVolts(double volts) {
        motor.setControl(voltage.withOutput(volts));
    }

    /** Declare the current position as fully down. */
    public void zero() {
        motor.setPosition(0);
    }

    public double units() {
        return -motor.getPosition().getValueAsDouble() * 360.0;
    }

    public double statorAmps() {
        return motor.getStatorCurrent().getValueAsDouble();
    }

    public void log() {
        DogLog.log("Hood/GoalUnits", goalUnits);
        DogLog.log("Hood/Units", units());
        DogLog.log("Hood/StatorAmps", statorAmps());
    }
}
