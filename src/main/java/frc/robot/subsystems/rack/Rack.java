package frc.robot.subsystems.rack;

// import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.util.LoggedTunableNumber;
import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;


public class Rack extends SubsystemBase{
    
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
    private final RackIO io;

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

    public Command setRackSpeed(double speed) {
        return runOnce(() -> this.io.setSpeed(speed));
    }

    public Command extendCmd() {
        return runOnce(() -> this.io.setVoltage(6));
    }
    public Command retractCmd() {
        return runOnce(() -> this.io.setVoltage(-6));
    }
    public Command stopCmd() {
        return runOnce(() -> this.io.setVoltage(0));
    }
    // /** 
    //  * Command to shoot out the coral
    //  */
    // public Command ejectCoralCmd(Elevator elevator) {
    //     double speed = defaultEjectSpeed;
    //     if (elevator.getNextStop() == ElevatorStop.L1) {
    //         speed = 0.25;
            
    //     } else if (elevator.getNextStop() == ElevatorStop.L4) {
    //         speed = 0.1;
    //     }  
    //     return this.setRackSpeed(speed);
    // }
    // public boolean hasCoral() {
    //     return this.inputs.supplyCurrent.gt(Amps.of(6.5));
    // }

    @Override
    public void periodic(){
        super.periodic();
        this.io.updateInputs(inputs);
        Logger.processInputs("Rack", inputs);
        //SmartDashboard.putBoolean("Has Coral?", this.hasCoral());
        SmartDashboard.putString("rack/motor voltage", this.inputs.appliedVolts.toString());
        SmartDashboard.putString("rack/motor supply current", this.inputs.supplyCurrent.toString());
        SmartDashboard.putString("rack/motor torque current", this.inputs.torqueCurrent.toString());
        SmartDashboard.putString("rack/motor temp", this.inputs.temperature.toString());        

    }
}
