package frc.robot.subsystems.rack;

import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutTemperature;
import edu.wpi.first.units.measure.MutVoltage;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.AutoLog;

public interface RackIO {

    @AutoLog
    class RackIOInputs {
        public boolean MotorConnected = true;
        public MutAngularVelocity velocity     = DegreesPerSecond.mutable(0);
        public MutVoltage appliedVolts         = Volts.mutable(0);
        public MutCurrent supplyCurrent        = Amps.mutable(0);
        public MutCurrent torqueCurrent        = Amps.mutable(0);
        public MutTemperature temperature      = Celsius.mutable(0);
    }

    default void updateInputs(RackIOInputs inputs) {}

    default void setPosition(double rotations) {}
    default void setSpeed(double speed) {}
    default void setVoltage(double volts) {}
    default void resetPosition(double knownRotations) {}
    default double getPosition() { return 0.0; }

    default void setPID(double p, double i, double d) {}
    default void setFF(double kS, double kG, double kV, double kA) {}
}