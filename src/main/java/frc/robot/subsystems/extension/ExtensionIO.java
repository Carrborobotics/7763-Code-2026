// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.extension;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ExtensionIO {
    @AutoLog
    public static class ExtensionIOInputs {
        public double extensionAngle = 0.0;
        public double extensionSetpoint = 0.0;
        public double extensionStatorCurrent = 0.0;
        public double extensionSupplyCurrent = 0.0;
        public double extensionVoltage = 0.0;
        public double extensionMotorTemp = 0.0;
        public boolean extensionMotorConnected = false;
        public boolean extensionEncoderConnected = false;
        public double extensionSpeed = 0.0;
    }

    public default void updateInputs(ExtensionIOInputs inputs) {}

    public default void setExtensionSetpoint(double position) {}
    
} 