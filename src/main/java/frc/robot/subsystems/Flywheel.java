package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Constants;
import frc.robot.Constants.CAN;
import frc.robot.Tuning;

/** Two shooter wheels on opposite sides, closed-loop velocity. 0 rps = coast, negative = reverse. */
public final class Flywheel {
    private final TalonFX left;
    private final TalonFX right;
    private final VelocityVoltage velocity = new VelocityVoltage(0);
    private final VoltageOut voltage = new VoltageOut(0);
    private final NeutralOut coast = new NeutralOut();
    private final MechanismSim.Motor simLeft;
    private final MechanismSim.Motor simRight;
    private double goalRps = 0;
    private double sysIdVolts = 0;

    public Flywheel() {
        var leftCfg = config(InvertedValue.CounterClockwise_Positive);
        var rightCfg = config(InvertedValue.Clockwise_Positive);
        left = Talons.create("Flywheel L", CAN.FLYWHEEL_LEFT, CAN.RIO, leftCfg);
        right = Talons.create("Flywheel R", CAN.FLYWHEEL_RIGHT, CAN.RIO, rightCfg);
        simLeft = MechanismSim.add(left, leftCfg, DCMotor.getKrakenX60(1), Constants.Sim.FLYWHEEL_MOI);
        simRight = MechanismSim.add(right, rightCfg, DCMotor.getKrakenX60(1), Constants.Sim.FLYWHEEL_MOI);
    }

    private static TalonFXConfiguration config(InvertedValue invert) {
        var cfg = Talons.config(invert, NeutralModeValue.Coast);
        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        cfg.CurrentLimits.SupplyCurrentLimit = 80;
        cfg.CurrentLimits.SupplyCurrentLowerLimit = 8;
        cfg.CurrentLimits.SupplyCurrentLowerTime = 0.5;
        cfg.Slot0.kP = Constants.Flywheel.KP;
        cfg.Slot0.kS = Constants.Flywheel.KS;
        cfg.Slot0.kV = Constants.Flywheel.KV;
        return cfg;
    }

    public void setRps(double rps) {
        goalRps = rps;
        if (rps == 0) {
            left.setControl(coast);
            right.setControl(coast);
        } else {
            left.setControl(velocity.withVelocity(rps));
            right.setControl(velocity.withVelocity(rps));
        }
    }

    /** Open-loop voltage for SysId characterization only. */
    public void setSysIdVolts(double volts) {
        goalRps = 0;
        sysIdVolts = volts;
        left.setControl(voltage.withOutput(volts));
        right.setControl(voltage.withOutput(volts));
    }

    public double sysIdVolts() {
        return sysIdVolts;
    }

    public double rps() {
        return left.getVelocity().getValueAsDouble();
    }

    public double rotations() {
        return left.getPosition().getValueAsDouble();
    }

    /** Average stator current of both motors. Spikes when a ball squeezes through. */
    public double statorAmps() {
        return (left.getStatorCurrent().getValueAsDouble() + right.getStatorCurrent().getValueAsDouble()) / 2;
    }

    public double goalRps() {
        return goalRps;
    }

    public boolean atSpeed() {
        return goalRps > 0 && Math.abs(rps() - goalRps) < Tuning.AT_SPEED_TOLERANCE_RPS.get();
    }

    /** Simulation only: a ball goes through and steals some speed (the controller then pulls current). */
    public void simBallThrough() {
        if (simLeft != null) simLeft.loseSpeed(Constants.Sim.BALL_SPEED_LOSS);
        if (simRight != null) simRight.loseSpeed(Constants.Sim.BALL_SPEED_LOSS);
    }

    public void log() {
        DogLog.log("Flywheel/GoalRps", goalRps);
        DogLog.log("Flywheel/LeftRps", rps());
        DogLog.log("Flywheel/RightRps", right.getVelocity().getValueAsDouble());
        DogLog.log("Flywheel/AtSpeed", atSpeed());
        DogLog.log("Flywheel/SupplyAmps", left.getSupplyCurrent().getValueAsDouble());
        DogLog.log("Flywheel/StatorAmps", statorAmps());
    }
}
