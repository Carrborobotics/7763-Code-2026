package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.math.system.plant.DCMotor;

import frc.robot.Constants;
import frc.robot.Constants.CAN;

/** One open-loop (percent output) motor. Used for the intake, the floor and the kicker. */
public final class Roller {
    private final String name;
    private final TalonFX motor;
    private final DutyCycleOut request = new DutyCycleOut(0);

    public Roller(String name, int canId, InvertedValue invert, NeutralModeValue neutral) {
        this.name = name;
        var cfg = Talons.config(invert, neutral);
        // Up to 80 A; only drops to 8 A after being pinned at 80 A for half a second (stall protection).
        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        cfg.CurrentLimits.SupplyCurrentLimit = 80;
        cfg.CurrentLimits.SupplyCurrentLowerLimit = 8;
        cfg.CurrentLimits.SupplyCurrentLowerTime = 0.5;
        motor = Talons.create(name, canId, CAN.RIO, cfg);
        MechanismSim.add(motor, cfg, DCMotor.getKrakenX60(1), Constants.Sim.ROLLER_MOI);
    }

    public void set(double duty) {
        motor.setControl(request.withOutput(duty));
    }

    public void log() {
        DogLog.log(name + "/Duty", request.Output);
        DogLog.log(name + "/VelocityRps", motor.getVelocity().getValueAsDouble());
        DogLog.log(name + "/SupplyAmps", motor.getSupplyCurrent().getValueAsDouble());
        DogLog.log(name + "/TempC", motor.getDeviceTemp().getValueAsDouble());
    }
}
