package frc.robot.subsystems.rack;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
//import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Distance;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
//import frc.robot.Robot;

import static edu.wpi.first.units.Units.*;

public class RackIOReal implements RackIO {
    private TalonFX rackMotorLeft;
    private TalonFX rackMotorRight;
    //private CANcoder rackEncoder;
    private CANcoder rackEncoder = new CANcoder(Constants.CANConstants.rackId, Constants.CANConstants.canBus);

    // Leaving these here in case we want voltage limits for the rack 
    // private final VoltageOut rackVoltage = new VoltageOut(2).withEnableFOC(true);
    // private final VoltageOut shootVoltage = new VoltageOut(2).withEnableFOC(true);
    // private final VoltageOut boostVoltage = new VoltageOut(2).withEnableFOC(true);

    private final MotionMagicVoltage m_request_extension = new MotionMagicVoltage(0);
    
    private double extensionSetpoint = 0.0;

    public RackIOReal() {


        var extensionPIDConfig = new Slot0Configs();
        extensionPIDConfig.GravityType = GravityTypeValue.Arm_Cosine;
        extensionPIDConfig.kS = 0.28;
        extensionPIDConfig.kV = 3.2;
        extensionPIDConfig.kA = 0.04;
        extensionPIDConfig.kP = 10; 
        extensionPIDConfig.kI = 0;
        extensionPIDConfig.kD = 0;

        // Create the rack kraken
        rackMotorLeft = new TalonFX(Constants.CANConstants.rackId, Constants.CANConstants.canBus);
        rackMotorRight = new TalonFX(Constants.CANConstants.rackId2, Constants.CANConstants.canBus);
        
        
        // Configure it differently than the swerves
        var rackConfig = new TalonFXConfiguration();


        rackConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        rackConfig.Slot0 = extensionPIDConfig;
        rackConfig.Feedback.FeedbackRemoteSensorID = rackEncoder.getDeviceID();
        rackConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        rackConfig.Feedback.RotorToSensorRatio = 5; //Constants.RACK_GEAR_RATIO;
        rackConfig.MotionMagic.MotionMagicCruiseVelocity = 100 / Constants.RACK_GEAR_RATIO;
        rackConfig.MotionMagic.MotionMagicAcceleration = rackConfig.MotionMagic.MotionMagicCruiseVelocity / 0.030;
        rackConfig.MotionMagic.MotionMagicExpo_kV = 0.12 * Constants.RACK_GEAR_RATIO;
        rackConfig.MotionMagic.MotionMagicExpo_kA = 0.1;
        rackConfig.ClosedLoopGeneral.ContinuousWrap = false;
        rackConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        rackConfig.CurrentLimits.StatorCurrentLimit = 10 ; //Constants.RACK_STATOR_CURRENT_LIMIT;
        rackConfig.CurrentLimits.SupplyCurrentLimit = 10 ; //Constants.RACK_SUPPLY_CURRENT_LIMIT;
        rackConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        rackConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        rackConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = !true;
        rackConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.32;

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


    // inputs.extensionSpeed = extensionMotor.getVelocity().getValueAsDouble();
    // inputs.extensionVoltage = extensionMotor.getMotorVoltage().getValueAsDouble();
    // inputs.extensionSetpoint = extensionSetpoint;
    //     rackMotorLeft.getC
    //     inputs.setpointPosition.mut_replace(Inches.of(rackMotorLeft.getClosedLoop().getValueAsDouble() * Constants.RACK_GEAR_RATIO));
    //     inputs.setpointVelocity.mut_replace(InchesPerSecond.of(rackMotorLeft.getClosedLoopTargetVelocity().getValueAsDouble() * Constants.RACK_GEAR_RATIO));
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

    @Override
    public void runSetpoint(Distance position) {
        double extensionSetpoint = position.in(Inches);
        rackMotorLeft.setControl(m_request_extension.withPosition(extensionSetpoint));
        rackMotorRight.setControl(m_request_extension.withPosition(extensionSetpoint));
    }

    public void periodic() {
    }

}

