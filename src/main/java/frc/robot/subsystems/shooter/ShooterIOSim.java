package frc.robot.subsystems.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import static edu.wpi.first.units.Units.*;

import frc.robot.Constants;

public class ShooterIOSim implements ShooterIO {
  DCMotorSim motor =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              DCMotor.getKrakenX60(1),
              0.001,
              Constants.SHOOTER_GEARING),
          DCMotor.getKrakenX60(1));


  @Override
  public void setVoltage(double v) {
    motor.setInputVoltage(v);
  }

  @Override
  public void setSpeed(double speed) {
    // In the real IO, setSpeed uses a percent output ([-1,1]) on the TalonFX.
    // Simulate that by driving the motor with an equivalent voltage (percent * 12V).
    // Previously this code forced the simulated angular velocity directly which
    // produced a short spike and then decayed toward physical behavior. Instead
    // set the input voltage so the motor model updates naturally.
    double volts = Math.max(-12.0, Math.min(12.0, speed * 12.0));
    motor.setInputVoltage(volts);
  }
  
  @Override
  public void updateInputs(ShooterIOInputs inputs) {
        motor.update(0.02); // 20ms update
        // motor.getAngularVelocity() returns radians/sec. The rest of the codebase
        // records shooter velocity as degrees/sec (see ShooterIOReal). Convert here
        // to keep logged units consistent.
        inputs.velocity.mut_replace(motor.getAngularVelocity());
        inputs.appliedVolts.mut_replace(motor.getInputVoltage(), Volts);
        inputs.supplyCurrent.mut_replace(motor.getCurrentDrawAmps(), Amps);
        inputs.torqueCurrent.mut_replace(motor.getCurrentDrawAmps(), Amps);
        inputs.temperature.mut_replace(0, Celsius);
        //inputs.setpointPosition.mut_replace(controller.getSetpoint(), Meters);
        //inputs.setpointVelocity.mut_replace(0, MetersPerSecond);
  }
}
