package frc.robot.subsystems.intake;
// import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.util.LoggedTunableNumber;
import static edu.wpi.first.units.Units.*;
import frc.robot.Constants;

import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase{
    
    // PID values
    private static final LoggedTunableNumber kP = new LoggedTunableNumber("Pivot/Gains/kP", 0.15);
    private static final LoggedTunableNumber kI = new LoggedTunableNumber("Pivot/Gains/kI", 0.0);
    private static final LoggedTunableNumber kD = new LoggedTunableNumber("Pivot/Gains/kD", 0.0);

    // Feedforward values
    private static final LoggedTunableNumber kS = new LoggedTunableNumber("Pivot/Gains/kS", 0.1);
    private static final LoggedTunableNumber kV = new LoggedTunableNumber("Pivot/Gains/kV", 1.45);
    private static final LoggedTunableNumber kA = new LoggedTunableNumber("Pivot/Gains/kA", 0.1);
    private static final LoggedTunableNumber kG = new LoggedTunableNumber("Pivot/Gains/kG", 0.1);
    
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    private final IntakeIO io;

    private final RobotState actual;
    private final RobotState target;
    private final RobotState goal;

    public Intake(IntakeIO io) {
        this.io = io;
        this.io.setPID(0.15, 0, 0);
        this.io.setPID(kP.get(), kI.get(), kD.get());
        this.io.setFF(kS.get(), kG.get(), kV.get(), kA.get());
        this.actual = RobotState.getMeasuredInstance();
        this.target = RobotState.getDesiredInstance();
        this.goal = RobotState.getGoalInstance();    
    }

    public Command setIntakeSpeed(double speed) {
        return runOnce(() -> this.io.setSpeed(speed));
    }

    public Command forwardCmd() {
        return this.setIntakeSpeed(Constants.Intake.FORWARD_SPEED);
    }
    
    public Command reverseCmd() {
        return this.setIntakeSpeed(Constants.Intake.REVERSE_SPEED);
    }

    /**
     *  Command to stop the intake
     */ 
    public Command stopCmd() {
        return runOnce(() -> this.io.setSpeed(0));
    }

    public boolean IsOverloaded() {
        return this.inputs.supplyCurrent.gt(Amps.of(6.5));
    }

    @Override
    public void periodic(){
        super.periodic();
        this.io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
        SmartDashboard.putBoolean("Is intake Overloaded?", this.IsOverloaded());
        SmartDashboard.putString("intake/intake motor voltage", this.inputs.appliedVolts.toString());
        SmartDashboard.putString("intake/intake supply current", this.inputs.supplyCurrent.toString());
        SmartDashboard.putString("intake/intake torque current", this.inputs.torqueCurrent.toString());
        SmartDashboard.putString("intake/intake motor temp", this.inputs.temperature.toString());   
        SmartDashboard.putString("intake/intake velocity", this.inputs.velocity.toString());     
    }
}
