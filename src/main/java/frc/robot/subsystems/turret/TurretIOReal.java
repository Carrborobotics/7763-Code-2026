package frc.robot.subsystems.turret;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
//import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

import static edu.wpi.first.units.Units.*;

/**
 * Real-world Turret IO using a CTRE TalonFX (KrakenX60) motor controller.
 */
public class TurretIOReal implements TurretIO {

    private final TalonFX motor;
    private final PositionVoltage positionRequest = new PositionVoltage(0);//.withEnableFOC(true);

    public TurretIOReal() {
        motor = new TalonFX(Constants.CANConstants.turretId, Constants.CANConstants.canBusDriveTrain);

        var config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Basic PID from TurretConstants (only kP/kI/kD applied to Slot0)
        config.Slot0.kP = 100.0; //TurretConstants.TalonFXGains.kP();
        config.Slot0.kI = 0.0; //TurretConstants.TalonFXGains.kI();
        config.Slot0.kD = 0.0; //TurretConstants.TalonFXGains.kD();
        config.Slot0.kS = 5.0; //TurretConstants.TalonFXGains.kS();
        config.Slot0.kV = 10.0; //TurretConstants.TalonFXGains.kV();
        // Configure sensor-to-mechanism ratio so CTRE scales between encoder rotations and
        // mechanism rotations (e.g. gearbox ratio). Use the config.Feedback field so
        // the configurator applies it to the controller.
        config.Feedback.SensorToMechanismRatio = 39.0;

        // Set invert and apply configuration
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        motor.getConfigurator().apply(config);

        // Zero the encoder
        try {
            motor.getConfigurator().setPosition(0.0);
        } catch (Exception ignored) {
            // best-effort; some environments may not support the timeout variant
            motor.setPosition(0.0);
        }
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        // Position/velocity from TalonFX are reported in rotations; convert to radians
        double positionRotations = motor.getPosition().getValueAsDouble();
        double velocityRps = motor.getVelocity().getValueAsDouble();

        inputs.position.mut_replace(edu.wpi.first.math.util.Units.rotationsToRadians(positionRotations), Radians);
        inputs.velocity.mut_replace(edu.wpi.first.math.util.Units.rotationsToRadians(velocityRps), RadiansPerSecond);

        inputs.appliedVolts.mut_replace(motor.getMotorVoltage().getValueAsDouble(), Volts);
        inputs.supplyCurrent.mut_replace(motor.getStatorCurrent().getValueAsDouble(), Amps);
        inputs.torqueCurrent.mut_replace(motor.getTorqueCurrent().getValueAsDouble(), Amps);
        inputs.temperature.mut_replace(motor.getDeviceTemp().getValueAsDouble(), Celsius);
    }

    @Override
    public void runSetpoint(Angle position) {
        // Convert Angle -> rotations (rotations = radians / 2pi)
        double rotations = position.in(Radians) / (2.0 * Math.PI);
        motor.setControl(positionRequest.withPosition(rotations));
    }

    @Override
    public void runVolts(Voltage volts) {
        // Use direct voltage output for open-loop commands
        motor.setVoltage(volts.in(Volts));
    }

    @Override
    public void setSpeed(double speed) {
        // Convert speed (in volts) to a voltage command
        motor.set(speed);
    }
}

