package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import static edu.wpi.first.units.Units.*;

import frc.robot.Constants;

public class IntakeIOSim implements IntakeIO {
  DCMotorSim motor =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              DCMotor.getKrakenX60(1),
              0.001,
              Constants.INTAKE_GEARING),
          DCMotor.getKrakenX60(1));


  @Override
  public void setVoltage(double v) {
    motor.setInputVoltage(v);
  }

  @Override
  public void setSpeed(double speed) {
    double rad_per_sec = speed * 2 * Math.PI; // Convert from rotations per second to radians per second
    motor.setAngularVelocity(rad_per_sec);
  }
  
  @Override
  public void updateInputs(IntakeIOInputs inputs) {
        motor.update(0.02); // 20ms update
        inputs.velocity.mut_replace(motor.getAngularVelocity());
        inputs.appliedVolts.mut_replace(motor.getInputVoltage(), Volts);
        inputs.supplyCurrent.mut_replace(motor.getCurrentDrawAmps(), Amps);
        inputs.torqueCurrent.mut_replace(motor.getCurrentDrawAmps(), Amps);
        inputs.temperature.mut_replace(0, Celsius);
        //inputs.setpointPosition.mut_replace(controller.getSetpoint(), Meters);
        //inputs.setpointVelocity.mut_replace(0, MetersPerSecond);


  }
}
