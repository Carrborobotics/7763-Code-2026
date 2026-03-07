package frc.robot;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
//import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
//import edu.wpi.first.units.measure.MutDistance;

import static edu.wpi.first.units.Units.*;

public class RobotState {
    private static RobotState measuredInstance;
    private static RobotState desiredInstance;
    private static RobotState goalInstance;

    private MutAngle rackPosition;
    private MutAngle turretPosition;
    private MutAngle shooterHoodPosition;

    private MutAngularVelocity intakeVelocity;
    private MutCurrent intakeSupplyCurrent;
    private MutCurrent intakeTorqueCurrent;

    private RobotState() {
        rackPosition = Degrees.mutable(0);
        turretPosition = Degrees.mutable(0);
        shooterHoodPosition = Degrees.mutable(0);
        intakeVelocity = DegreesPerSecond.mutable(0);
        intakeSupplyCurrent = Amps.mutable(0);
        intakeTorqueCurrent = Amps.mutable(0);
    }

    public static RobotState getMeasuredInstance() {
        if (measuredInstance == null) {
            measuredInstance = new RobotState();
        }
        return measuredInstance;
    }

    public static RobotState getDesiredInstance() {
        if (desiredInstance == null) {
            desiredInstance = new RobotState();
        }
        return desiredInstance;
    }

    public static RobotState getGoalInstance() {
        if (goalInstance == null) {
            goalInstance = new RobotState();
        }
        return goalInstance;
    }

    public Angle getRackPosition() {
        return rackPosition;
    }

    public void updateRackAngle(Angle position) {
        rackPosition.mut_replace(position);
    }

    public Angle getTurretPosition() {
        return turretPosition;
    }
    
    public void updateTurretAngle(Angle position) {
        turretPosition.mut_replace(position);
    }

    public AngularVelocity getIntakeVelocity(){
        return intakeVelocity;
    }

    public Current getIntakeSupplyCurrent(){
        return intakeSupplyCurrent;
    }
   
    public Angle getShooterHoodPosition() {
        return shooterHoodPosition;
    }

    public void updateShooterHoodAngle(Angle position) {
        shooterHoodPosition.mut_replace(position);
    }

    public Current getIntakeTorqueCurrent(){
        return intakeTorqueCurrent;
    }
}
