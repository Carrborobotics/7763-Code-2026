package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.math.system.plant.DCMotor;

import frc.robot.Constants;
import frc.robot.Constants.CAN;

/**
 * Turret yaw. Angles are robot-relative degrees, CCW positive, 0 = robot forward. The turret MUST be
 * pointing forward when the robot boots, because that is where the encoder is zeroed.
 */
public final class Turret {
    private final TalonFX motor;
    private final PositionVoltage position = new PositionVoltage(0);
    private double goalDeg = 0;

    public Turret() {
        var cfg = Talons.config(InvertedValue.CounterClockwise_Positive, NeutralModeValue.Brake);
        cfg.Voltage.PeakForwardVoltage = Constants.Turret.PEAK_VOLTS;
        cfg.Voltage.PeakReverseVoltage = -Constants.Turret.PEAK_VOLTS;
        cfg.Slot0.kP = Constants.Turret.KP;
        cfg.Slot0.kS = Constants.Turret.KS;
        cfg.Slot0.kV = Constants.Turret.KV;
        cfg.Feedback.SensorToMechanismRatio = Constants.Turret.SENSOR_TO_MECHANISM;
        motor = Talons.create("Turret", CAN.TURRET, CAN.DRIVETRAIN, cfg);
        MechanismSim.add(motor, cfg, DCMotor.getKrakenX60(1), Constants.Sim.TURRET_MOI);
        motor.setPosition(0);
    }

    public void setDegrees(double deg) {
        goalDeg = deg;
        // Motor positive is clockwise when viewed from above, so the sign flips.
        motor.setControl(position.withPosition(-deg / 360.0));
    }

    public double degrees() {
        return -motor.getPosition().getValueAsDouble() * 360.0;
    }

    public void log() {
        DogLog.log("Turret/GoalDeg", goalDeg);
        DogLog.log("Turret/Deg", degrees());
        DogLog.log("Turret/StatorAmps", motor.getStatorCurrent().getValueAsDouble());
        // Match Logs app: Mechanisms tab and 3D view joints
        DogLog.log("Mechanisms/Turret/Position", degrees(), "deg");
        DogLog.log("Mechanisms/Turret/Setpoint", goalDeg, "deg");
    }
}
