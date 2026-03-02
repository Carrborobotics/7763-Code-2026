package frc.robot.subsystems.rack;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
//import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
//import frc.robot.Robot;

import static edu.wpi.first.units.Units.*;

public class RackIOReal implements RackIO {
    private TalonFX rackMotorLeft;
    private TalonFX rackMotorRight;
    private CANcoder rackEncoder;

    // Leaving these here in case we want voltage limits for the rack 
    // private final VoltageOut rackVoltage = new VoltageOut(2).withEnableFOC(true);
    // private final VoltageOut shootVoltage = new VoltageOut(2).withEnableFOC(true);
    // private final VoltageOut boostVoltage = new VoltageOut(2).withEnableFOC(true);

    public RackIOReal() {
        // Create the rack kraken
        rackMotorLeft = new TalonFX(Constants.CANConstants.rackId, Constants.CANConstants.canBus);
        rackMotorRight = new TalonFX(Constants.CANConstants.rackId2, Constants.CANConstants.canBus);
        
        
        // Configure it differently than the swerves
        var rackConfig = new TalonFXConfiguration();

        // Brake mode
        rackConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        // Make sure current limiting is enabled
        rackConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        // Limit the current to the rack to max 70A
        rackConfig.CurrentLimits.SupplyCurrentLimit = 10;

        // Wait for time at limit then lower the limit - useful to not brownout 
        rackConfig.CurrentLimits.SupplyCurrentLowerLimit = 8;
        
        // How long to wait before throttling
        rackConfig.CurrentLimits.SupplyCurrentLowerTime = 0.5;

        // Keep the voltages within range
        rackConfig.Voltage.PeakForwardVoltage = 12;
        rackConfig.Voltage.PeakReverseVoltage = 12;

        rackMotorLeft.getConfigurator().apply(rackConfig);
        rackMotorRight.getConfigurator().apply(rackConfig);

        // Not really used except for tracking velocity but keep jic
        rackEncoder = new CANcoder(Constants.CANConstants.rackId, Constants.CANConstants.canBus);
    }

    @Override
    public void updateInputs(RackIOInputs inputs) {
        
        inputs.velocity.mut_replace(rackEncoder.getVelocity().getValueAsDouble(), DegreesPerSecond);
        

        inputs.appliedVolts.mut_replace(rackMotorLeft.getSupplyVoltage().getValueAsDouble(), Volts);

        inputs.supplyCurrent.mut_replace(rackMotorLeft.getSupplyCurrent().getValueAsDouble(), Amps);

        inputs.torqueCurrent.mut_replace(rackMotorLeft.getTorqueCurrent().getValueAsDouble(), Amps);

        inputs.temperature.mut_replace(rackMotorLeft.getDeviceTemp().getValueAsDouble(), Celsius);
        


        inputs.appliedVolts.mut_replace(rackMotorRight.getSupplyVoltage().getValueAsDouble(), Volts);

        inputs.supplyCurrent.mut_replace(rackMotorRight.getSupplyCurrent().getValueAsDouble(), Amps);

        inputs.torqueCurrent.mut_replace(rackMotorRight.getTorqueCurrent().getValueAsDouble(), Amps);

        inputs.temperature.mut_replace(rackMotorRight.getDeviceTemp().getValueAsDouble(), Celsius);
    }

    @Override
    public void setSpeed(double speed) {
        rackMotorLeft.set(speed);
        rackMotorRight.set(speed);
    }

    @Override 
    public void setVoltage(double volts) {
        rackMotorLeft.setVoltage(volts);
        rackMotorRight.setVoltage(volts);
    }

    public void periodic() {
    }

}

