package frc.robot.subsystems.rack;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import static edu.wpi.first.units.Units.*;
import frc.robot.Constants;

public class RackIOSim implements RackIO {
  DCMotorSim motorA =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60(1), 
            0.001,
            Constants.RACK_GEARING),
            DCMotor.getKrakenX60(1));

  DCMotorSim motorB =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60(1),
            0.001,
            Constants.RACK_GEARING),
          DCMotor.getKrakenX60(1));

  @Override
  public void setVoltage(double v) {
    motorA.setInputVoltage(v);
    motorB.setInputVoltage(v);
  }

  @Override
  public void updateInputs(RackIOInputs inputs) {
      motorA.update(0.02); // 20ms update
      motorB.update(0.02); // 20ms update
    
      inputs.velocity.mut_replace(motorA.getAngularVelocity());
      inputs.appliedVolts.mut_replace(motorA.getInputVoltage(), Volts);
      inputs.supplyCurrent.mut_replace(motorA.getCurrentDrawAmps(), Amps);
      inputs.torqueCurrent.mut_replace(motorA.getCurrentDrawAmps(), Amps);
      inputs.temperature.mut_replace(0, Celsius);
  }
}
