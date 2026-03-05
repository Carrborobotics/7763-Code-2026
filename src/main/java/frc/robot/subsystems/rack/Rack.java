package frc.robot.subsystems.rack;

import java.lang.annotation.ElementType;
import java.util.EnumMap;
import java.util.Map;

import org.littletonrobotics.junction.Logger;

//import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
//import frc.robot.subsystems.elevator.Elevator;
//import frc.robot.subsystems.elevator.Elevator.ElevatorStop;
import frc.robot.util.LoggedTunableNumber;

import static edu.wpi.first.units.Units.Degrees;

public class Rack extends SubsystemBase {
    
    // PID values
    private static final LoggedTunableNumber kP = new LoggedTunableNumber("Rack/Gains/kP", 0.15);
    private static final LoggedTunableNumber kI = new LoggedTunableNumber("Rack/Gains/kI", 0.0);
    private static final LoggedTunableNumber kD = new LoggedTunableNumber("Rack/Gains/kD", 0.0);
    
    // Feedforward values
    private static final LoggedTunableNumber kS = new LoggedTunableNumber("Rack/Gains/kS", 0.1);
    private static final LoggedTunableNumber kV = new LoggedTunableNumber("Rack/Gains/kV", 1.45);
    private static final LoggedTunableNumber kA = new LoggedTunableNumber("Rack/Gains/kA", 0.1);
    private static final LoggedTunableNumber kG = new LoggedTunableNumber("Rack/Gains/kG", 0.1);

    private final RackIO io;
    private final RackIOInputsAutoLogged inputs = new RackIOInputsAutoLogged();
    
    private Angle setpoint = Degrees.of(0.0);
    private Angle position = Degrees.mutable(0.0);

    private final RobotState actual;
    private final RobotState target;
    private final RobotState goal;


    public Rack(RackIO io) {
        this.io = io;
        this.io.setPID(0.15, 0, 0);
        this.io.setPID(kP.get(), kI.get(), kD.get());
        this.io.setFF(kS.get(), kG.get(), kV.get(), kA.get());
        this.actual = RobotState.getMeasuredInstance();
        this.target = RobotState.getDesiredInstance();
        this.goal = RobotState.getGoalInstance();
    }
    
    public Command rackToCmd(double degree_value) {
        return Commands.runOnce(() -> this.setpoint = Degrees.of(degree_value));
    }
    public Command setPosition(Angle position) {
        return runOnce(() -> this.setpoint = Degrees.of(position.in(Degrees)));
    }
    public Command setSpeedCmd(double val) {
        return runOnce(() -> this.io.setSpeed(val));
    }   

    @Override
    public void periodic() {
        super.periodic();
        position = this.actual.getRackPosition();
        this.io.updateInputs(inputs);
        Logger.processInputs("Rack", inputs);
        SmartDashboard.putNumber("Rack/position(actual)", this.inputs.position.in(Degrees));
        SmartDashboard.putNumber("Rack/Position2", position.in(Degrees));
        SmartDashboard.putNumber("Rack/setpointPosition(tgt)", this.inputs.setpointPosition.in(Degrees));
        SmartDashboard.putNumber("Rack/setpoint(goal)", this.setpoint.in(Degrees));
        if(edu.wpi.first.wpilibj.RobotState.isDisabled()) {
            this.io.stop();
        } else {
            this.io.runSetpoint(this.setpoint);
            this.inputs.setpointPosition.mut_replace(this.setpoint);
        }
        actual.updateRackAngle(this.inputs.position);
        target.updateRackAngle(this.inputs.setpointPosition);
        goal.updateRackAngle(this.setpoint);
    }
}
