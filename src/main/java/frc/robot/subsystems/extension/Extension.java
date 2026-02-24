// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.extension;


import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Extension extends SubsystemBase {
  private ExtensionIO io;
  private ExtensionIOInputsAutoLogged inputs = new ExtensionIOInputsAutoLogged();
  private final Alert ExtensionDisconnectedAlert;
  /** Creates a new Extension. */
  public Extension(ExtensionIO io) {
    this.io = io;
    ExtensionDisconnectedAlert = new Alert("Extension motor disconnected", AlertType.kError);
  }
    public void setExtensionSetpoint(double extensionSetpoint) {
      io.setExtensionSetpoint(extensionSetpoint);
  }

  public double extensionSetpoint() {
    return inputs.extensionSetpoint;
  }

  @Override
  public void periodic() {
    
    io.updateInputs(inputs);
    //Logger.processInputs("Extension/Io", inputs);
    //ExtensionDisconnectedAlert.set(!inputs.extensionMotorConnected);
    // This method will be called once per scheduler run
  }
}
