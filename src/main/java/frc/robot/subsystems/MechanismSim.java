package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

/**
 * Desktop simulation for mechanism TalonFXs: each motor gets a simple DC-motor physics model, so
 * positions, velocities and currents move realistically in sim. Does nothing on the real robot.
 */
public final class MechanismSim {
    private MechanismSim() {}

    /** One simulated motor. Keep the handle if the mechanism needs to disturb its sim (e.g. a ball hit). */
    public static final class Motor {
        private final TalonFXSimState state;
        private final DCMotorSim physics;
        private final double gearing;

        private Motor(TalonFX motor, TalonFXConfiguration cfg, DCMotor model, double moiKgM2) {
            state = motor.getSimState();
            state.Orientation = cfg.MotorOutput.Inverted == InvertedValue.Clockwise_Positive
                    ? ChassisReference.Clockwise_Positive : ChassisReference.CounterClockwise_Positive;
            gearing = cfg.Feedback.SensorToMechanismRatio;
            physics = new DCMotorSim(LinearSystemId.createDCMotorSystem(model, moiKgM2, gearing), model);
        }

        private void step(double dtS) {
            state.setSupplyVoltage(RobotController.getBatteryVoltage());
            physics.setInputVoltage(state.getMotorVoltage());
            physics.update(dtS);
            state.setRawRotorPosition(physics.getAngularPositionRotations() * gearing);
            state.setRotorVelocity(physics.getAngularVelocityRPM() / 60.0 * gearing);
        }

        /** Instantly lose a fraction of speed (a simulated ball squeezing through). */
        public void loseSpeed(double fraction) {
            physics.setState(physics.getAngularPositionRad(), physics.getAngularVelocityRadPerSec() * (1 - fraction));
        }
    }

    private static final List<Motor> MOTORS = new ArrayList<>();

    /** Simulate this motor (sim only). Returns null on the real robot. */
    public static Motor add(TalonFX motor, TalonFXConfiguration cfg, DCMotor model, double moiKgM2) {
        if (!RobotBase.isSimulation()) return null;
        Motor m = new Motor(motor, cfg, model, moiKgM2);
        MOTORS.add(m);
        return m;
    }

    public static void update() {
        for (Motor m : MOTORS) m.step(Constants.LOOP_PERIOD_S);
    }
}
