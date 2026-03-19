package frc.robot.subsystems.turret;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.util.LoggedTunableNumber;

import static edu.wpi.first.units.Units.Degrees;

public class Turret extends SubsystemBase {
    
    // PID values
    private static final LoggedTunableNumber kP = new LoggedTunableNumber("Turret/Gains/kP", 0.15);
    private static final LoggedTunableNumber kI = new LoggedTunableNumber("Turret/Gains/kI", 0.0);
    private static final LoggedTunableNumber kD = new LoggedTunableNumber("Turret/Gains/kD", 0.0);
    
    // Feedforward values
    private static final LoggedTunableNumber kS = new LoggedTunableNumber("Turret/Gains/kS", 0.1);
    private static final LoggedTunableNumber kV = new LoggedTunableNumber("Turret/Gains/kV", 1.45);
    private static final LoggedTunableNumber kA = new LoggedTunableNumber("Turret/Gains/kA", 0.1);
    private static final LoggedTunableNumber kG = new LoggedTunableNumber("Turret/Gains/kG", 0.1);

    private final TurretIO io;
    private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
    
    private Angle setpoint = Degrees.of(0.0);
    private Angle position = Degrees.mutable(0.0);

    private final RobotState actual;
    private final RobotState target;
    private final RobotState goal;


    public Turret(TurretIO io) {
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
    public void setTurretAngle(double degrees) {
        this.setpoint = Degrees.of(degrees);
    }

    public Command setSpeedCmd(double val) {
        return runOnce(() -> this.io.setSpeed(val));
    }   

    public Command modifyOffsetCmd(double offset) {
        return runOnce(() -> this.io.modifyOffset(offset));
    }

    public double getTargetTurretDegrees() {
        return this.setpoint.in(Degrees);
    }

    @Override
    public void periodic() {
        super.periodic();
        position = this.actual.getTurretPosition();
        this.io.updateInputs(inputs);
        Logger.processInputs("Turret", inputs);
        SmartDashboard.putNumber("Turret/position(actual)", this.inputs.position.in(Degrees));
        SmartDashboard.putNumber("Turret/Position2", position.in(Degrees));
        SmartDashboard.putNumber("Turret/setpointPosition(tgt)", this.inputs.setpointPosition.in(Degrees));
        SmartDashboard.putNumber("Turret/setpoint(goal)", this.setpoint.in(Degrees));
        if(edu.wpi.first.wpilibj.RobotState.isDisabled()) {
            this.io.stop();
        } else {
            this.io.runSetpoint(this.setpoint);
            this.inputs.setpointPosition.mut_replace(this.setpoint);
        }
        actual.updateTurretAngle(this.inputs.position);
        target.updateTurretAngle(this.inputs.setpointPosition);
        goal.updateTurretAngle(this.setpoint);
    }
}
