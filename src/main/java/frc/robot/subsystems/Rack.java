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
 * Rack-and-pinion intake deploy, two motors each running their own position loop to the same goal.
 * The rack MUST be fully retracted at boot, because that is where the encoders are zeroed.
 */
public final class Rack {
    private final TalonFX a;
    private final TalonFX b;
    private final PositionVoltage position = new PositionVoltage(0);
    private double goalRot = 0;

    public Rack() {
        var cfg = Talons.config(InvertedValue.CounterClockwise_Positive, NeutralModeValue.Brake);
        cfg.Slot0.kP = Constants.Rack.KP;
        a = Talons.create("Rack A", CAN.RACK_A, CAN.RIO, cfg);
        b = Talons.create("Rack B", CAN.RACK_B, CAN.RIO, cfg);
        MechanismSim.add(a, cfg, DCMotor.getKrakenX60(1), Constants.Sim.RACK_MOI);
        MechanismSim.add(b, cfg, DCMotor.getKrakenX60(1), Constants.Sim.RACK_MOI);
        a.setPosition(0);
        b.setPosition(0);
    }

    public void setRotations(double rot) {
        goalRot = rot;
        a.setControl(position.withPosition(rot));
        b.setControl(position.withPosition(rot));
    }

    public void log() {
        DogLog.log("Rack/GoalRot", goalRot);
        DogLog.log("Rack/RotA", a.getPosition().getValueAsDouble());
        DogLog.log("Rack/RotB", b.getPosition().getValueAsDouble());
        DogLog.log("Rack/StatorAmpsA", a.getStatorCurrent().getValueAsDouble());
        // Match Logs app: Mechanisms tab and 3D view joints (motor rotations, 0 = retracted)
        DogLog.log("Mechanisms/Rack/Position", a.getPosition().getValueAsDouble(), "rot");
        DogLog.log("Mechanisms/Rack/Setpoint", goalRot, "rot");
    }
}
