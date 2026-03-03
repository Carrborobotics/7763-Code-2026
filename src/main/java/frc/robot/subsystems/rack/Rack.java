package frc.robot.subsystems.rack;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Rack extends SubsystemBase {

    // ── Setpoints (motor rotations) ───────────────────────────────────────────
    public static final class Setpoints {
        public static final double RETRACTED = 0.0;  // TODO: tune
        public static final double DEPLOYED  = 10.0; // TODO: tune
        public static final double PARTIAL   = 5.0;  // TODO: tune
    }

    private static final double POSITION_TOLERANCE_ROTATIONS = 0.2;

    private final RackIO io;
    private final RackIOInputsAutoLogged inputs = new RackIOInputsAutoLogged();

    public Rack(RackIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Rack", inputs);
        //SmartDashboard.putBoolean("Is Overloaded?", this.IsOverloaded());
        SmartDashboard.putString("rack/motor voltage", this.inputs.appliedVolts.toString());
        SmartDashboard.putString("rack/motor supply current", this.inputs.supplyCurrent.toString());
        SmartDashboard.putString("rack/motor torque current", this.inputs.torqueCurrent.toString());
        SmartDashboard.putString("rack/motor temp", this.inputs.temperature.toString());      
        SmartDashboard.putString("rack/position", String.format("%.2f", getPosition()));
    }

    // ── Public API ────────────────────────────────────────────────────────────

    public void setPosition(double rotations) { io.setPosition(rotations); }
    public void retract()  { setPosition(Setpoints.RETRACTED); }
    public void  deploy()   { setPosition(Setpoints.DEPLOYED);  }
    public void partial()  { setPosition(Setpoints.PARTIAL);   }
    public void stop()     { io.setSpeed(0); }
    public Command setSpeed(double speed) { return runOnce(() -> io.setSpeed(speed)); }

    public void resetPosition(double knownRotations) { io.resetPosition(knownRotations); }

    public double getPosition() { return io.getPosition(); }
    public boolean atSetpoint(double targetRotations) {
        return Math.abs(getPosition() - targetRotations) < POSITION_TOLERANCE_ROTATIONS;
    }

    // ── Command factories ─────────────────────────────────────────────────────

    public Command deployCmd()  { return new InstantCommand(() -> deploy(),  this); }
    public Command retractCmd() { return new InstantCommand(() -> retract(), this); }
    public Command partialCmd() { return new InstantCommand(() -> partial(), this); }
    public Command stopCmd()    { return new InstantCommand(() -> stop(),    this); }

    /**
     * Command to reset the encoder to a known rotation value (default 0).
     */
    public Command resetEncoderCmd() { return new InstantCommand(() -> resetPosition(0.0), this); }

    /**
     * Command to reset the encoder to a specified known rotations value.
     */
    public Command resetEncoderCmd(double knownRotations) { return new InstantCommand(() -> resetPosition(knownRotations), this); }
}