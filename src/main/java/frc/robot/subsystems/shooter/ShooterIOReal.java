package frc.robot.subsystems.shooter;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
//import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
//import frc.robot.Robot;

import static edu.wpi.first.units.Units.*;

public class ShooterIOReal implements ShooterIO {
    private TalonFX shooterMotor;
    private TalonFX shooterMotor2;
    private CANcoder shooterEncoder;
    //private CANcoder shooterEncoder2;
    private TalonFX kickerMotor;
    //private TalonFX hoodMotor;
    

    // Leaving these here in case we want voltage limits for the shooter 
    // private final VoltageOut shooterVoltage = new VoltageOut(2).withEnableFOC(true);
    // private final VoltageOut shootVoltage = new VoltageOut(2).withEnableFOC(true);
    // private final VoltageOut boostVoltage = new VoltageOut(2).withEnableFOC(true);
    final VelocityVoltage m_velocity  = new VelocityVoltage(0);
    double reqSpeed = 0;

    public ShooterIOReal() {
        // Create the shooter kraken
        shooterMotor = new TalonFX(Constants.CANConstants.shooterLeft, Constants.CANConstants.canBus);
        shooterMotor2 = new TalonFX(Constants.CANConstants.shooterRight, Constants.CANConstants.canBus);
        kickerMotor = new TalonFX(Constants.CANConstants.kickerId, Constants.CANConstants.canBus);
        //hoodMotor = new TalonFX(Constants.CANConstants.hoodId, Constants.CANConstants.canBus);


        // Configure it differently than the swerves
        var shooterConfig = new TalonFXConfiguration();
        // var hoodConfig = new TalonFXConfiguration();

        // Brake mode
        shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        //hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        // Make sure current limiting is enabled
        shooterConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        //hoodConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        // Limit the current to the shooter to max 70A
        shooterConfig.CurrentLimits.SupplyCurrentLimit = 80;
        //oodConfig.CurrentLimits.SupplyCurrentLimit = 80;

        // Wait for time at limit then lower the limit - useful to not brownout 
        shooterConfig.CurrentLimits.SupplyCurrentLowerLimit = 8;
        //hoodConfig.CurrentLimits.SupplyCurrentLowerLimit = 8;
        
        // How long to wait before throttling
        shooterConfig.CurrentLimits.SupplyCurrentLowerTime = 0.5;
        //hoodConfig.CurrentLimits.SupplyCurrentLowerTime = 0.5;

        // Keep the voltages within range
        shooterConfig.Voltage.PeakForwardVoltage = 12;
        shooterConfig.Voltage.PeakReverseVoltage = 12;
        //hoodConfig.Voltage.PeakForwardVoltage = 12;
        //hoodConfig.Voltage.PeakReverseVoltage = 12;
        shooterConfig.Slot0.kP = 0.1;
        shooterConfig.Slot0.kI = 0.0;
        shooterConfig.Slot0.kD = 0.0;
        shooterConfig.Slot0.kS = 0.0;
        shooterConfig.Slot0.kV = 0.0;

        shooterMotor.getConfigurator().apply(shooterConfig);
        kickerMotor.getConfigurator().apply(shooterConfig);
        //hoodMotor.getConfigurator().apply(hoodConfig);
        shooterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        shooterMotor2.getConfigurator().apply(shooterConfig);

        m_velocity.Slot = 0;

        // Not really used except for tracking velocity but keep jic
        shooterEncoder = new CANcoder(Constants.CANConstants.shooterLeft, Constants.CANConstants.canBus);
        //shooterEncoder2 = new CANcoder(Constants.CANConstants.shooterRight, Constants.CANConstants.canBus);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        
        inputs.velocity.mut_replace(shooterMotor.getVelocity().getValueAsDouble(), DegreesPerSecond);

        inputs.kickerVelocity.mut_replace(kickerMotor.getVelocity().getValueAsDouble(), DegreesPerSecond);

        inputs.appliedVolts.mut_replace(shooterMotor.getSupplyVoltage().getValueAsDouble(), Volts);

        inputs.supplyCurrent.mut_replace(shooterMotor.getSupplyCurrent().getValueAsDouble(), Amps);

        inputs.torqueCurrent.mut_replace(shooterMotor.getTorqueCurrent().getValueAsDouble(), Amps);

        inputs.temperature.mut_replace(shooterMotor.getDeviceTemp().getValueAsDouble(), Celsius);
    }

    @Override
    public void setSpeed(double speed) {
        if (speed < 1) {
            shooterMotor.stopMotor();
            shooterMotor2.stopMotor();
            kickerMotor.stopMotor();
        }
        else {
            shooterMotor.setControl(m_velocity.withVelocity(speed));
            shooterMotor2.setControl(m_velocity.withVelocity(speed));
            kickerMotor.setControl(m_velocity.withVelocity(speed));
        }
    }


    @Override
    public void setVoltage(double volts) {
        shooterMotor.setVoltage(volts);
        shooterMotor2.setVoltage(volts);
        kickerMotor.setVoltage(volts);
    }
    public void periodic() {

    }

}

