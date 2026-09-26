package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DriverStation;

/** Shared TalonFX setup so every motor is configured the same, reliable way. */
final class Talons {
    private Talons() {}

    static TalonFXConfiguration config(InvertedValue invert, NeutralModeValue neutral) {
        var cfg = new TalonFXConfiguration();
        cfg.MotorOutput.Inverted = invert;
        cfg.MotorOutput.NeutralMode = neutral;
        return cfg;
    }

    /**
     * Create a TalonFX, apply its config (retrying, because CAN can drop frames at boot) and register it
     * with {@link Health} under a human-readable name.
     */
    static TalonFX create(String name, int id, CANBus bus, TalonFXConfiguration cfg) {
        var motor = new TalonFX(id, bus);
        StatusCode status = StatusCode.OK;
        for (int attempt = 0; attempt < 5; attempt++) {
            status = motor.getConfigurator().apply(cfg);
            if (status.isOK()) break;
        }
        if (!status.isOK()) {
            DriverStation.reportError(name + " (TalonFX " + id + ") config failed: " + status, false);
        }
        Health.register(name + " (" + id + ")", motor);
        return motor;
    }
}
