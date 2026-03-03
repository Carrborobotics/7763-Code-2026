package frc.robot.subsystems.rack;

import org.littletonrobotics.junction.Logger;

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
    }

    // ── Public API ────────────────────────────────────────────────────────────

    public void setPosition(double rotations) { io.setPosition(rotations); }
    public void retract()  { setPosition(Setpoints.RETRACTED); }
    public void deploy()   { setPosition(Setpoints.DEPLOYED);  }
    public void partial()  { setPosition(Setpoints.PARTIAL);   }
    public void stop()     { io.setSpeed(0); }

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
}