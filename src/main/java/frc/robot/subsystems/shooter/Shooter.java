package frc.robot.subsystems.shooter;
// import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.util.LoggedTunableNumber;
import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase{
    
    // PID values
    private static final LoggedTunableNumber kP = new LoggedTunableNumber("Pivot/Gains/kP", 0.15);
    private static final LoggedTunableNumber kI = new LoggedTunableNumber("Pivot/Gains/kI", 0.0);
    private static final LoggedTunableNumber kD = new LoggedTunableNumber("Pivot/Gains/kD", 0.0);

    // Feedforward values
    private static final LoggedTunableNumber kS = new LoggedTunableNumber("Pivot/Gains/kS", 0.1);
    private static final LoggedTunableNumber kV = new LoggedTunableNumber("Pivot/Gains/kV", 1.45);
    private static final LoggedTunableNumber kA = new LoggedTunableNumber("Pivot/Gains/kA", 0.1);
    private static final LoggedTunableNumber kG = new LoggedTunableNumber("Pivot/Gains/kG", 0.1);
    
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    private final ShooterIO io;

    private final RobotState actual;
    private final RobotState target;
    private final RobotState goal;

    public Shooter(ShooterIO io) {
        this.io = io;
        this.io.setPID(0.15, 0, 0);
        this.io.setPID(kP.get(), kI.get(), kD.get());
        this.io.setFF(kS.get(), kG.get(), kV.get(), kA.get());
        this.actual = RobotState.getMeasuredInstance();
        this.target = RobotState.getDesiredInstance();
        this.goal = RobotState.getGoalInstance();    
    }

    public Command setShooterSpeed(double speed) {
        return runOnce(() -> this.io.setSpeed(speed));
    }

    /**
     *  Command to stop the shooter
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
        Logger.processInputs("Shooter", inputs);
        SmartDashboard.putBoolean("Is shooter Overloaded?", this.IsOverloaded());
        SmartDashboard.putString("shooter/shooter voltage", this.inputs.appliedVolts.toString());
        SmartDashboard.putString("shooter/shooter supply current", this.inputs.supplyCurrent.toString());
        SmartDashboard.putString("shooter/shooter torque current", this.inputs.torqueCurrent.toString());
        SmartDashboard.putString("shooter/shooter motor temp", this.inputs.temperature.toString());   
        SmartDashboard.putString("shooter/shooter velocity", this.inputs.velocity.toString());  
        SmartDashboard.putString("shooter/kicker velocity", this.inputs.kickerVelocity.toString());   
    }
}
