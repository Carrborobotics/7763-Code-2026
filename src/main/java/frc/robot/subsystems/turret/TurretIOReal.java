package frc.robot.subsystems.turret;


import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
//import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

import static edu.wpi.first.units.Units.*;


public class TurretIOReal implements TurretIO {
   
    private SparkFlex leader;
    private SparkClosedLoopController closedLoopControllerTurret;
    

    public TurretIOReal() {
        SparkFlexConfig config = new SparkFlexConfig();
        SparkFlex turretSpark = new SparkFlex(Constants.CANConstants.turretId, MotorType.kBrushless);
        config
                .inverted(false) // left: inverted=true, right: inverted=false
                .idleMode(IdleMode.kBrake);
        config.encoder
                .positionConversionFactor(1)
                .velocityConversionFactor(1);
        config.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(0.1, 0.0, 0.0);
        turretSpark.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        turretSpark.getEncoder().setPosition(0);
        closedLoopControllerTurret = turretSpark.getClosedLoopController();
    
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        inputs.position.mut_replace(leader.getAbsoluteEncoder().getPosition(), Degrees);
        inputs.appliedVoltsLeader.mut_replace(leader.getBusVoltage(), Volts);
        inputs.supplyCurrentLeader.mut_replace(leader.getOutputCurrent(), Amps);
        inputs.torqueCurrentLeader.mut_replace(leader.getOutputCurrent(), Amps);
        inputs.temperatureLeader.mut_replace(leader.getMotorTemperature(), Celsius);
    }

    @Override
    public void runSetpoint(Angle position) {
        double setpoint = position.in(Degree);
        closedLoopControllerTurret.setReference(setpoint, SparkFlex.ControlType.kPosition);
    }

    @Override
    public void runVolts(Voltage volts) {
        leader.setVoltage(volts.in(Volts));
    }

}

