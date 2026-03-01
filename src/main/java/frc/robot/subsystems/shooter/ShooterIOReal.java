package frc.robot.subsystems.shooter;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
//import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
//import frc.robot.Robot;

import static edu.wpi.first.units.Units.*;

public class ShooterIOReal implements ShooterIO {
    private TalonFX shooterMotor;
    private CANcoder shooterEncoder;

    // Leaving these here in case we want voltage limits for the shooter 
    // private final VoltageOut shooterVoltage = new VoltageOut(2).withEnableFOC(true);
    // private final VoltageOut shootVoltage = new VoltageOut(2).withEnableFOC(true);
    // private final VoltageOut boostVoltage = new VoltageOut(2).withEnableFOC(true);

    public ShooterIOReal() {
        // Create the shooter kraken
        shooterMotor = new TalonFX(Constants.CANConstants.shooterId, Constants.CANConstants.canBus);
        
        // Configure it differently than the swerves
        var shooterConfig = new TalonFXConfiguration();

        // Brake mode
        shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        // Make sure current limiting is enabled
        shooterConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        // Limit the current to the shooter to max 70A
        shooterConfig.CurrentLimits.SupplyCurrentLimit = 10;

        // Wait for time at limit then lower the limit - useful to not brownout 
        shooterConfig.CurrentLimits.SupplyCurrentLowerLimit = 8;
        
        // How long to wait before throttling
        shooterConfig.CurrentLimits.SupplyCurrentLowerTime = 0.5;

        // Keep the voltages within range
        shooterConfig.Voltage.PeakForwardVoltage = 12;
        shooterConfig.Voltage.PeakReverseVoltage = 12;

        shooterMotor.getConfigurator().apply(shooterConfig);

        // Not really used except for tracking velocity but keep jic
        shooterEncoder = new CANcoder(Constants.CANConstants.shooterId, Constants.CANConstants.canBus);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        
        inputs.velocity.mut_replace(shooterEncoder.getVelocity().getValueAsDouble(), DegreesPerSecond);

        inputs.appliedVolts.mut_replace(shooterMotor.getSupplyVoltage().getValueAsDouble(), Volts);

        inputs.supplyCurrent.mut_replace(shooterMotor.getSupplyCurrent().getValueAsDouble(), Amps);

        inputs.torqueCurrent.mut_replace(shooterMotor.getTorqueCurrent().getValueAsDouble(), Amps);

        inputs.temperature.mut_replace(shooterMotor.getDeviceTemp().getValueAsDouble(), Celsius);
    }

    @Override
    public void setSpeed(double speed) {
        shooterMotor.set(speed);
    }

    @Override
    public void setVoltage(double volts) {
        shooterMotor.setVoltage(volts);
    }
    public void periodic() {
    }

}

