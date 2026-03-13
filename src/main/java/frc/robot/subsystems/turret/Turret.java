package frc.robot.subsystems.turret;

//import java.lang.annotation.ElementType;
//import java.util.EnumMap;
//import java.util.Map;

import org.littletonrobotics.junction.Logger;

//import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.units.measure.Angle;
//import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.util.Color;
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
    
    // TODO: Next 3 things are all basically the same command, we should probably unify them
    public Command turretToCmd(double degree_value) {
        return Commands.runOnce(() -> this.setpoint = Degrees.of(degree_value));
    }
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
    
    // private void __configureTurret() {
    //     TalonFXConfiguration cfg = new TalonFXConfiguration();

    //     // PID gains
    //     cfg.Slot0.kP = TURRET_kP;
    //     cfg.Slot0.kI = TURRET_kI;
    //     cfg.Slot0.kD = TURRET_kD;
    //     cfg.Slot0.kS = TURRET_kS;

    //     // MotionMagic profile — smooth trapezoidal motion to setpoint
    //     cfg.MotionMagic.MotionMagicCruiseVelocity = TURRET_MOTION_CRUISE_RPS;
    //     cfg.MotionMagic.MotionMagicAcceleration   = TURRET_MOTION_ACCEL_RPS2;

    //     // Soft limits
    //     cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    //     cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    //     // Soft limits are set in motor shaft rotations, so we convert degrees → rotations.
    //     // Formula: limitRotations = limitDegrees / TURRET_DEGREES_PER_ROTATION
    //     cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = degreesToRotations(TURRET_MAX_DEGREES, TURRET_DEGREES_PER_ROTATION);
    //     cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = degreesToRotations(TURRET_MIN_DEGREES, TURRET_DEGREES_PER_ROTATION);

    //     cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    //     // Kraken X60 current limits
    //     cfg.CurrentLimits.StatorCurrentLimit = TURRET_STATOR_LIMIT;
    //     cfg.CurrentLimits.StatorCurrentLimitEnable = true;

    //     turretMotor.getConfigurator().apply(cfg);
    // }

    // private void setupShuffleboard() {
    //     tab.addDouble("Shooter RPM (Target)",   () -> targetShooterRPM);
    //     tab.addDouble("Shooter Lead RPM",       () -> getShooterRPM());
    //     tab.addDouble("Shooter Follow RPM",     () -> rpsToRPM(shooterFollow.getVelocity().getValueAsDouble()));
    //     tab.addBoolean("Shooter At Speed",      () -> isShooterAtSpeed());
    //     tab.addDouble("Hood Angle (Target)",    () -> targetHoodDegrees);
    //     tab.addDouble("Hood Angle (Actual)",    () -> getHoodDegrees());
    //     tab.addBoolean("Hood At Angle",         () -> isHoodAtAngle());
    //     tab.addDouble("Turret Angle (Target)",  () -> targetTurretDegrees);
    //     tab.addDouble("Turret Angle (Actual)",  () -> getTurretDegrees());
    //     tab.addBoolean("Turret At Angle",       () -> isTurretAtAngle());
    //     tab.addBoolean("Ready To Shoot",        () -> isReadyToShoot());
    // }

    // // ── Public API ─────────────────────────────────────────────────────────────

    // /**
    //  * Set the shooter flywheel to a target RPM.
    //  * @param rpm Target RPM (0 to stop)
    //  */
    // public void setShooterRPM(double rpm) {
    //     targetShooterRPM = rpm;
    //     if (rpm <= 0) {
    //         shooterLead.stopMotor();
    //         // Follower will coast since it follows lead
    //     } else {
    //         // Phoenix 6 VelocityVoltage takes RPS (rotations per second), not RPM.
    //         // Formula: RPS = RPM / 60
    //         shooterLead.setControl(shooterControl.withVelocity(rpmToRPS(rpm)));
    //     }
    // }

    // /** Stop the shooter flywheel. */
    // public void stopShooter() {
    //     setShooterRPM(0);
    // }

    // /**
    //  * Set the hood to a target angle in degrees.
    //  * @param degrees Target angle (clamped to soft limits automatically)
    //  */
    // public void setHoodAngle(double degrees) {
    //     targetHoodDegrees = clamp(degrees, HOOD_MIN_DEGREES, HOOD_MAX_DEGREES);
    //     // PositionVoltage takes motor shaft rotations, not degrees.
    //     // Formula: rotations = degrees / HOOD_DEGREES_PER_ROTATION
    //     hoodMotor.setControl(hoodControl.withPosition(
    //         degreesToRotations(targetHoodDegrees, HOOD_DEGREES_PER_ROTATION)));
    // }

    // /**
    //  * Set the turret rotation to a target angle in degrees (0 = forward).
    //  * @param degrees Target angle (clamped to soft limits automatically)
    //  */

    // public void setTurretAngle(double degrees) {
    //     targetTurretDegrees = clamp(degrees, TURRET_MIN_DEGREES, TURRET_MAX_DEGREES);
    //     // MotionMagicVoltage takes motor shaft rotations, not degrees.
    //     // Formula: rotations = degrees / TURRET_DEGREES_PER_ROTATION
    //     turretMotor.setControl(turretControl.withPosition(
    //         degreesToRotations(targetTurretDegrees, TURRET_DEGREES_PER_ROTATION)));
    // }

    // /** Zero the turret encoder at the current position (call when at known home). */
    // public void zeroTurret() {
    //     turretMotor.setPosition(0.0);
    // }

    // /** Zero the hood encoder at the current position (call when at hard stop).
    //  *  Seeds the encoder to HOOD_MIN_DEGREES worth of rotations so position
    //  *  reads correctly from the start. Formula: rotations = minDegrees / degreesPerRotation */
    // public void zeroHood() {
    //     hoodMotor.setPosition(degreesToRotations(HOOD_MIN_DEGREES, HOOD_DEGREES_PER_ROTATION));
    // }

    // // ── State queries ──────────────────────────────────────────────────────────

    // // getVelocity() returns motor shaft RPS — multiply by 60 to get RPM
    // public double getShooterRPM() {
    //     return rpsToRPM(shooterLead.getVelocity().getValueAsDouble());
    // }

    // // getPosition() returns motor shaft rotations — multiply by degreesPerRotation to get output degrees
    // public double getHoodDegrees() {
    //     return rotationsToDegrees(hoodMotor.getPosition().getValueAsDouble(), HOOD_DEGREES_PER_ROTATION);
    // }

    // // getPosition() returns motor shaft rotations — multiply by degreesPerRotation to get output degrees
    // public double getTurretDegrees() {
    //     return rotationsToDegrees(turretMotor.getPosition().getValueAsDouble(), TURRET_DEGREES_PER_ROTATION);
    // }

    public double getTargetTurretDegrees() {
        //return targetTurretDegrees;
        return this.setpoint.in(Degrees); // TODO: Aidan, this may not be correct...
    }

    // public boolean isShooterAtSpeed() {
    //     return targetShooterRPM > 0
    //         && Math.abs(getShooterRPM() - targetShooterRPM) < SHOOTER_RPM_TOLERANCE;
    // }

    // public boolean isHoodAtAngle() {
    //     return Math.abs(getHoodDegrees() - targetHoodDegrees) < HOOD_TOLERANCE_DEGREES;
    // }

    // public boolean isTurretAtAngle() {
    //     return Math.abs(getTurretDegrees() - targetTurretDegrees) < TURRET_TOLERANCE_DEGREES;
    // }

    // /** Returns true when all three mechanisms are on target and ready to fire. */
    // public boolean isReadyToShoot() {
    //     return isShooterAtSpeed() && isHoodAtAngle() && isTurretAtAngle();
    // }

    // ── Periodic ───────────────────────────────────────────────────────────────

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
