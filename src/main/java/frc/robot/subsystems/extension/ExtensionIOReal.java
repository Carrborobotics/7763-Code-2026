// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.extension;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.Constants;

/** Add your docs here. */ 
public class ExtensionIOReal implements ExtensionIO{
  private CANcoder extensionEncoder = new CANcoder(Constants.CANConstants.rackId2, Constants.CANConstants.canBus);
  private TalonFX extensionMotor = new TalonFX(Constants.CANConstants.rackId2, Constants.CANConstants.canBus);

  private final StatusSignal<Angle> extensionAngle;
  private final StatusSignal<Current> extensionStatorCurrent;
  private final StatusSignal<Current> extensionSupplyCurrent;
  private final StatusSignal<AngularVelocity> extensionSpeed;
  private final StatusSignal<Voltage> extensionVoltage;
  private final StatusSignal<Temperature> extensionTemp;

  private double extensionSetpoint = 0.0;
  private double extensionFeedForward = 0.0;

  private final MotionMagicVoltage m_request_extension = new MotionMagicVoltage(0);
  private final Alert extensionAlert = 
      new Alert( "Extension Motor Disconnected.", AlertType.kWarning );
  private final Alert extensionEncoderAlert =
      new Alert("Extension Encoder Disconnected.", AlertType.kWarning);

  public ExtensionIOReal() {

    var ExtensionPIDConfig = new Slot0Configs();
    ExtensionPIDConfig.GravityType = GravityTypeValue.Arm_Cosine;
    // pivotPIDConfig.kS = 0.28;
    ExtensionPIDConfig.kV = 0;
    ExtensionPIDConfig.kA = 0;
    ExtensionPIDConfig.kP = 75; 
    ExtensionPIDConfig.kI = 0;
    ExtensionPIDConfig.kD = 0;

    var cancoderConfig = new CANcoderConfiguration();
    cancoderConfig.MagnetSensor.withMagnetOffset(0);
    cancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    tryUntilOk(5, () -> extensionEncoder.getConfigurator().apply(cancoderConfig, 0.25));

    var extensionConfig = new TalonFXConfiguration();
    extensionConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    extensionConfig.Slot0 = ExtensionPIDConfig;
    extensionConfig.Feedback.RotorToSensorRatio = Constants.RACK_GEARING;
    extensionConfig.Feedback.FeedbackRemoteSensorID = extensionEncoder.getDeviceID();
    extensionConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    extensionConfig.MotionMagic.MotionMagicCruiseVelocity = 100 / Constants.RACK_GEARING;
    extensionConfig.MotionMagic.MotionMagicAcceleration =
    extensionConfig.MotionMagic.MotionMagicCruiseVelocity / 0.050;
    extensionConfig.MotionMagic.MotionMagicExpo_kV = 0.12 * Constants.RACK_GEARING;
    extensionConfig.MotionMagic.MotionMagicExpo_kA = 0.1;
    extensionConfig.ClosedLoopGeneral.ContinuousWrap = false;
    extensionConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    extensionConfig.CurrentLimits.StatorCurrentLimit = 10; //Constants.RACK_STATOR_CURRENT_LIMIT;
    extensionConfig.CurrentLimits.SupplyCurrentLimit = 10; //Constants.RACK_SUPPLY_CURRENT_LIMIT;
    extensionConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    extensionConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    tryUntilOk(5, () -> extensionMotor.getConfigurator().apply(extensionConfig, 0.25));

    extensionAngle = extensionEncoder.getPosition();
    extensionStatorCurrent = extensionMotor.getStatorCurrent();
    extensionSupplyCurrent = extensionMotor.getSupplyCurrent();
    extensionSpeed = extensionMotor.getVelocity();
    extensionVoltage = extensionMotor.getMotorVoltage();
    extensionTemp = extensionMotor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        extensionAngle,
        extensionStatorCurrent,
        extensionSupplyCurrent,
        extensionSpeed,
        extensionVoltage,
        extensionTemp);

    ParentDevice.optimizeBusUtilizationForAll(extensionMotor);

  }

@Override
  public void updateInputs(ExtensionIOInputs inputs) {
    var ExtensionStatus =
        BaseStatusSignal.refreshAll(
            extensionAngle,
            extensionStatorCurrent,
            extensionSupplyCurrent,
            extensionSpeed,
            extensionVoltage,
            extensionTemp);


    inputs.extensionMotorConnected = ExtensionStatus.isOK();
    inputs.extensionStatorCurrent = extensionStatorCurrent.getValueAsDouble();
    inputs.extensionSupplyCurrent = extensionSupplyCurrent.getValueAsDouble();
    inputs.extensionSpeed = extensionMotor.getVelocity().getValueAsDouble();
    inputs.extensionVoltage = extensionMotor.getMotorVoltage().getValueAsDouble();
    inputs.extensionSetpoint = extensionSetpoint;

    extensionAlert.set(!inputs.extensionMotorConnected);
    extensionEncoderAlert.set(!inputs.extensionEncoderConnected);

  }


   @Override
  public void setExtensionSetpoint(double position) {
    extensionSetpoint = Units.degreesToRotations(position);
    extensionMotor.setControl(m_request_extension.withPosition(extensionSetpoint));
  }
}
