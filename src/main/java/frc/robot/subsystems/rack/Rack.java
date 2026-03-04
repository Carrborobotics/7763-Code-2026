package frc.robot.subsystems.rack;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutDistance;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotState;
import frc.robot.util.LoggedTunableNumber;

import static edu.wpi.first.units.Units.Inches;

public class Rack extends SubsystemBase {
 
    private Distance setpoint = Inches.of(0.0);
    private Distance position = Inches.mutable(0.0);

    private final RobotState actual;
    private final RobotState target;
    private final RobotState goal;

    // ── Setpoints (motor rotations) ───────────────────────────────────────────
    public static final class Setpoints {
        public static final double RETRACTED = 0.0;  // TODO: tune
        public static final double DEPLOYED  = 10.0; // TODO: tune
        public static final double PARTIAL   = 5.0;  // TODO: tune
    }

      // PID values
    private static final LoggedTunableNumber kP = new LoggedTunableNumber("Turret/Gains/kP", 0.15);
    private static final LoggedTunableNumber kI = new LoggedTunableNumber("Turret/Gains/kI", 0.0);
    private static final LoggedTunableNumber kD = new LoggedTunableNumber("Turret/Gains/kD", 0.0);
    
    // Feedforward values
    private static final LoggedTunableNumber kS = new LoggedTunableNumber("Turret/Gains/kS", 0.1);
    private static final LoggedTunableNumber kV = new LoggedTunableNumber("Turret/Gains/kV", 1.45);
    private static final LoggedTunableNumber kA = new LoggedTunableNumber("Turret/Gains/kA", 0.1);
    private static final LoggedTunableNumber kG = new LoggedTunableNumber("Turret/Gains/kG", 0.1);

    //private static final double POSITION_TOLERANCE_INCHES = 0.2;

    private final RackIO io;
    private final RackIOInputsAutoLogged inputs = new RackIOInputsAutoLogged();

    public Rack(RackIO io) {
        this.io = io;
        this.io.setPID(0.15, 0, 0);
        this.io.setPID(kP.get(), kI.get(), kD.get());
        this.io.setFF(kS.get(), kG.get(), kV.get(), kA.get());
        this.actual = RobotState.getMeasuredInstance();
        this.target = RobotState.getDesiredInstance();
        this.goal = RobotState.getGoalInstance();
    }


    public Command rackToCmd(double dist) {
        return Commands.runOnce(() -> this.setpoint = Inches.of(dist));
    }
    public Command setPosition(Distance position) {
        return runOnce(() -> this.setpoint = Inches.of(position.in(Inches)));
    }

    @Override
    public void periodic() {
        super.periodic();
        position = this.actual.getRackPosition();
        io.updateInputs(inputs);
        Logger.processInputs("Rack", inputs);
        //SmartDashboard.putBoolean("Is Overloaded?", this.IsOverloaded());
        SmartDashboard.putString("Rack/motor voltage", this.inputs.appliedVolts.toString());
        SmartDashboard.putString("Rack/motor supply current", this.inputs.supplyCurrent.toString());
        SmartDashboard.putString("Rack/motor torque current", this.inputs.torqueCurrent.toString());
        SmartDashboard.putString("Rack/motor temp", this.inputs.temperature.toString());      
        SmartDashboard.putString("Rack/position", String.format("%.2f", getPosition()));
        SmartDashboard.putString("Rack/Position2", this.inputs.position.toString());
        SmartDashboard.putString("Rack/Position", position.toString());
        SmartDashboard.putString("Rack/setpointPosition", this.inputs.setpointPosition.toString());
        SmartDashboard.putString("Rack/setpoint", this.setpoint.toString());


        if(edu.wpi.first.wpilibj.RobotState.isDisabled()) {
            this.io.stop();
        } else {
            //this.io.runSetpoint(this.setpoint); // TODO: this is very broken and commented out
            this.inputs.setpointPosition.mut_replace(this.setpoint);
        }

        actual.updateRackPosition(this.inputs.position);
        target.updateRackPosition(this.inputs.setpointPosition);
        goal.updateRackPosition(this.setpoint);

    }

    // ── Public API ────────────────────────────────────────────────────────────

    public void setPosition(double rotations) { io.setPosition(rotations); }
    public void retract()  { setPosition(Setpoints.RETRACTED); }
    public void deploy()   { setPosition(Setpoints.DEPLOYED);  }
    public void partial()  { setPosition(Setpoints.PARTIAL);   }
    public void stop()     { io.setSpeed(0); }
    public Command setSpeed(double speed) { return runOnce(() -> io.setSpeed(speed)); }

    public void resetPosition(double knownRotations) { io.resetPosition(knownRotations); }

    public double getPosition() { return io.getPosition(); }
    // public boolean atSetpoint(double targetRotations) {
    //     return Math.abs(getPosition() - targetRotations) < POSITION_TOLERANCE_ROTATIONS;
    // }

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