package frc.robot.subsystems.floor;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
//import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
//import frc.robot.Robot;

import static edu.wpi.first.units.Units.*;

public class FloorIOReal implements FloorIO {
    private TalonFX floorMotor;
    private CANcoder floorEncoder;

    // Leaving these here in case we want voltage limits for the floor 
    // private final VoltageOut floorVoltage = new VoltageOut(2).withEnableFOC(true);
    // private final VoltageOut shootVoltage = new VoltageOut(2).withEnableFOC(true);
    // private final VoltageOut boostVoltage = new VoltageOut(2).withEnableFOC(true);

    public FloorIOReal() {
        // Create the floor kraken
        floorMotor = new TalonFX(Constants.CANConstants.floorId, Constants.CANConstants.canBus);
        
        // Configure it differently than the swerves
        var floorConfig = new TalonFXConfiguration();

        // Brake mode
        floorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        // Make sure current limiting is enabled
        floorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        // Limit the current to the floor to max 70A
        floorConfig.CurrentLimits.SupplyCurrentLimit = 10;

        // Wait for time at limit then lower the limit - useful to not brownout 
        floorConfig.CurrentLimits.SupplyCurrentLowerLimit = 8;
        
        // How long to wait before throttling
        floorConfig.CurrentLimits.SupplyCurrentLowerTime = 0.5;

        // Keep the voltages within range
        floorConfig.Voltage.PeakForwardVoltage = 12;
        floorConfig.Voltage.PeakReverseVoltage = 12;

        floorMotor.getConfigurator().apply(floorConfig);

        // Not really used except for tracking velocity but keep jic
        floorEncoder = new CANcoder(Constants.CANConstants.floorId, Constants.CANConstants.canBus);
    }

    @Override
    public void updateInputs(FloorIOInputs inputs) {
        
        inputs.velocity.mut_replace(floorEncoder.getVelocity().getValueAsDouble(), DegreesPerSecond);

        inputs.appliedVolts.mut_replace(floorMotor.getSupplyVoltage().getValueAsDouble(), Volts);

        inputs.supplyCurrent.mut_replace(floorMotor.getSupplyCurrent().getValueAsDouble(), Amps);

        inputs.torqueCurrent.mut_replace(floorMotor.getTorqueCurrent().getValueAsDouble(), Amps);

        inputs.temperature.mut_replace(floorMotor.getDeviceTemp().getValueAsDouble(), Celsius);
    }

    @Override
    public void setSpeed(double speed) {
        floorMotor.set(speed);
    }

    @Override
    public void setVoltage(double volts) {
        floorMotor.setVoltage(volts);
    }
    public void periodic() {
    }

}

