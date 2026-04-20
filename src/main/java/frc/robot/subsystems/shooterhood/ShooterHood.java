package frc.robot.subsystems.shooterhood;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.util.LoggedTunableNumber;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;

public class ShooterHood extends SubsystemBase {
    
    // PID values
    private static final LoggedTunableNumber kP = new LoggedTunableNumber("ShooterHood/Gains/kP", 0.15);
    private static final LoggedTunableNumber kI = new LoggedTunableNumber("ShooterHood/Gains/kI", 0.0);
    private static final LoggedTunableNumber kD = new LoggedTunableNumber("ShooterHood/Gains/kD", 0.0);
    
    // Feedforward values
    private static final LoggedTunableNumber kS = new LoggedTunableNumber("ShooterHood/Gains/kS", 0.1);
    private static final LoggedTunableNumber kV = new LoggedTunableNumber("ShooterHood/Gains/kV", 1.45);
    private static final LoggedTunableNumber kA = new LoggedTunableNumber("ShooterHood/Gains/kA", 0.1);
    private static final LoggedTunableNumber kG = new LoggedTunableNumber("ShooterHood/Gains/kG", 0.1);

    private final ShooterHoodIO io;
    private final ShooterHoodIOInputsAutoLogged inputs = new ShooterHoodIOInputsAutoLogged();
    
    private Angle setpoint = Degrees.of(0.0);
    private Angle position = Degrees.mutable(0.0);

    private final RobotState actual;
    private final RobotState target;
    private final RobotState goal;


    public ShooterHood(ShooterHoodIO io) {
        this.io = io;
        this.io.setPID(0.15, 0, 0);
        this.io.setPID(kP.get(), kI.get(), kD.get());
        this.io.setFF(kS.get(), kG.get(), kV.get(), kA.get());
        this.actual = RobotState.getMeasuredInstance();
        this.target = RobotState.getDesiredInstance();
        this.goal = RobotState.getGoalInstance();
    }
    
    // TODO: Next 2 things are all basically the same command, we should probably unify them
    public Command setPosition(Angle position) {
        return runOnce(() -> this.setpoint = Degrees.of(position.in(Degrees)));
    }
    public void setShooterHoodAngle(double degrees) {
        this.setpoint = Degrees.of(degrees);
    }

    public Command setSpeedCmd(double val) {
        return runOnce(() -> this.io.setSpeed(val));
    }   

    public double getTargetShooterHoodDegrees() {
        return this.setpoint.in(Degrees);
    }

    public boolean IsOverloaded() {
        return this.inputs.supplyCurrent.gt(Amps.of(5));
    }

    @Override
    public void periodic() {
        super.periodic();
        position = this.actual.getShooterHoodPosition();
        this.io.updateInputs(inputs);
        Logger.processInputs("ShooterHood", inputs);
        SmartDashboard.putNumber("ShooterHood/hood position(actual)", this.inputs.position.in(Degrees));
        SmartDashboard.putNumber("ShooterHood/hood Position2", position.in(Degrees));
        SmartDashboard.putNumber("ShooterHood/hood setpointPosition(tgt)", this.inputs.setpointPosition.in(Degrees));
        SmartDashboard.putNumber("ShooterHood/hood setpoint(goal)", this.setpoint.in(Degrees));
        SmartDashboard.putString("hood current", this.inputs.supplyCurrent.toString());
        if(edu.wpi.first.wpilibj.RobotState.isDisabled()) {
            this.io.stop();
        } else {
            this.io.runSetpoint(this.setpoint);
            this.inputs.setpointPosition.mut_replace(this.setpoint);
        }
        actual.updateShooterHoodAngle(this.inputs.position);
        target.updateShooterHoodAngle(this.inputs.setpointPosition);
        goal.updateShooterHoodAngle(this.setpoint);
    }
}
