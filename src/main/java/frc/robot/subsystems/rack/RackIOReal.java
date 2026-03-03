package frc.robot.subsystems.rack;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
//import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
//import frc.robot.Robot;

public class RackIOReal implements RackIO {
    private TalonFX rackMotorLeft;
    private TalonFX rackMotorRight;
    private CANcoder rackEncoder;
    private MotionMagicVoltage m_mmRequest = new MotionMagicVoltage(0);

    public static final class Setpoints {
        public static final double RETRACTED  = 0.0;   // TODO: tune
        public static final double DEPLOYED   = 10.0;  // TODO: tune
    }

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

         // --- Soft limits (protect your mechanism) ---
        rackConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable    = true;
        rackConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 10.5; // TODO: tune
        rackConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable    = true;
        rackConfig  .SoftwareLimitSwitch.ReverseSoftLimitThreshold = -0.5; // TODO: tune

        // --- Slot 0 PID gains ---
        // Start conservative; tune kP first, then kD, then kS/kV/kA
        rackConfig.Slot0.kP = 2.4;   // TODO: tune — output per rotation of error
        rackConfig.Slot0.kI = 0.0;
        rackConfig.Slot0.kD = 0.1;   // TODO: tune
        rackConfig.Slot0.kS = 0.25;  // TODO: tune — static friction (volts)
        rackConfig.Slot0.kV = 0.12;  // TODO: tune — volts per RPS
        rackConfig.Slot0.kA = 0.01;  // TODO: tune — volts per RPS²

        // --- MotionMagic cruise / acceleration ---
        rackConfig.MotionMagic.MotionMagicCruiseVelocity = 80;   // RPS   TODO: tune
        rackConfig.MotionMagic.MotionMagicAcceleration   = 160;  // RPS/s TODO: tune
        rackConfig.MotionMagic.MotionMagicJerk            = 1600; // RPS/s² — smooths the profile

        // Apply config to leader
        rackMotorLeft.getConfigurator().apply(rackConfig);

        // Follower mirrors the leader (set opposeLeaderDirection=true if it's
        // physically mirrored and needs to spin the opposite direction)
        rackMotorRight.setControl(new Follower(rackMotorLeft.getDeviceID(), MotorAlignmentValue.Aligned));

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

     /** Move to an arbitrary position (motor rotations). */
    public void setPosition(double rotations) {
        rackMotorLeft.setControl(m_mmRequest.withPosition(rotations));
    }

    public void retract()  {
        setPosition(Setpoints.RETRACTED); 
    }

    public void deploy()   {
        setPosition(Setpoints.DEPLOYED);  
    }

    /** Stop the motor and let brake mode hold position. */
    public void stop() {
        rackMotorLeft.stopMotor();
    }

    /** Seed the encoder — call this once at match start if you have a known home. */
    public void resetPosition(double knownRotations) {
        rackMotorLeft.setPosition(knownRotations);
    }

    public void periodic() {
    }

}

