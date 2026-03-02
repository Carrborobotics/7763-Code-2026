package frc.robot.subsystems.turret;

import java.lang.annotation.ElementType;
import java.util.EnumMap;
import java.util.Map;

import org.littletonrobotics.junction.Logger;

//import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.units.measure.Angle;
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

    private final RobotState actual;
    private final RobotState target;
    private final RobotState goal;


    public Turret(TurretIO io) {
        this.io = io;
        //this.io.setPID(0.15, 0, 0);
        this.io.setPID(kP.get(), kI.get(), kD.get());
        this.io.setFF(kS.get(), kG.get(), kV.get(), kA.get());
        this.actual = RobotState.getMeasuredInstance();
        this.target = RobotState.getDesiredInstance();
        this.goal = RobotState.getGoalInstance();
    }
    
    public Command turretTo(double degree_value) {
        return Commands.runOnce(() -> this.setpoint = Degrees.of(degree_value));
    }

    // /**
    //  * Turn turret to the correct Shoot-angle based on the elevator setting.
    //  * @param stop the ElevatorStop
    //  */
    // public Command turretToOnElevator(ElevatorStop stop) {
    //     return Commands.runOnce( 
    //         () -> {
    //             if (stop == ElevatorStop.L4) {
    //                 this.setpoint = turretsPos.get(Turrets.ShootL4);
    //             }
    //             else if (stop == ElevatorStop.L1) {
    //                 this.setpoint = turretsPos.get(Turrets.ShootL1);
    //             }
    //             else {
    //                 this.setpoint = turretsPos.get(Turrets.Shoot);    
    //             }
    //         }
    //     );
    // }

    public Command setPosition(Angle position) {
        return runOnce(() -> this.setpoint = position);
    }

    // public boolean isTurretSafe() {
    //     return (setpoint.compareTo(turretsPos.get(Turrets.Up)) > -0.5);
    // }

    @Override
    public void periodic() {
        super.periodic();

        this.io.updateInputs(inputs);
        Logger.processInputs("Turret", inputs);
        SmartDashboard.putString("Turret/Position", this.inputs.position.toString());

        if(edu.wpi.first.wpilibj.RobotState.isDisabled()) {
            this.io.stop();
        } else {
            this.io.runSetpoint(this.setpoint);
        }

        actual.updateTurretAngle(this.inputs.position);
        target.updateTurretAngle(this.inputs.setpointPosition);
        goal.updateTurretAngle(this.setpoint);

    }

    

}
