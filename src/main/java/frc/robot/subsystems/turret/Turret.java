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
import frc.robot.subsystems.rack.RackIO;
import frc.robot.subsystems.rack.RackIOInputsAutoLogged;
//import frc.robot.subsystems.elevator.Elevator;
//import frc.robot.subsystems.elevator.Elevator.ElevatorStop;
import frc.robot.util.LoggedTunableNumber;

import static edu.wpi.first.units.Units.*;

public class Turret extends SubsystemBase {
    // PID values
    private static final LoggedTunableNumber kP = new LoggedTunableNumber("Pivot/Gains/kP", 0.15);
    private static final LoggedTunableNumber kI = new LoggedTunableNumber("Pivot/Gains/kI", 0.0);
    private static final LoggedTunableNumber kD = new LoggedTunableNumber("Pivot/Gains/kD", 0.0);

    // Feedforward values
    private static final LoggedTunableNumber kS = new LoggedTunableNumber("Pivot/Gains/kS", 0.1);
    private static final LoggedTunableNumber kV = new LoggedTunableNumber("Pivot/Gains/kV", 1.45);
    private static final LoggedTunableNumber kA = new LoggedTunableNumber("Pivot/Gains/kA", 0.1);
    private static final LoggedTunableNumber kG = new LoggedTunableNumber("Pivot/Gains/kG", 0.1);
    
    private final RackIOInputsAutoLogged inputs = new RackIOInputsAutoLogged();
    private final TurretIO io;

    private Angle setpoint = Degrees.of(0.0);

    private final RobotState actual;
    private final RobotState target;
    private final RobotState goal;

    private boolean moveToSetPoint = false;


    public Turret(TurretIO io) {
        this.io = io;

        this.io.setPID(0.15, 0, 0);
        this.io.setPID(kP.get(), kI.get(), kD.get());
        this.io.setFF(kS.get(), kG.get(), kV.get(), kA.get());
        this.actual = RobotState.getMeasuredInstance();
        this.target = RobotState.getDesiredInstance();
        this.goal = RobotState.getGoalInstance();
    }

   

    public Command pivotToHorizontal(Angle angle) {
        return Commands.runOnce(() -> this.setpoint = angle);
    }
    public Command pivotToVertical(Angle angle) {
        return Commands.runOnce(() -> this.setpoint = angle);
    }
    
     /**
      * Turn pivot to the correct Shoot-angle based on the elevator setting.
      * @param stop the ElevatorStop
      */

          @Override
    public void periodic() {
        super.periodic();

        if(moveToSetPoint)
        {
            
        }
    }


}
