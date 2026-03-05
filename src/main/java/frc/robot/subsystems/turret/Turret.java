package frc.robot.subsystems.turret;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {

    // ── CAN IDs ────────────────────────────────────────────────────────────────
    private static final int SHOOTER_LEAD_CAN_ID   = 10; // primary shooter motor
    private static final int SHOOTER_FOLLOW_CAN_ID = 13; // mirrored follower (inverted)
    private static final int HOOD_CAN_ID            = 11; // Kraken X44
    private static final int TURRET_CAN_ID          = 12; // Kraken X60

    // ── Current limits ─────────────────────────────────────────────────────────
    // X60 handles higher continuous stator current than X44 — limits reflect that
    private static final double SHOOTER_STATOR_LIMIT = 60.0; // amps — adjust to your shooter motor type
    private static final double TURRET_STATOR_LIMIT  = 80.0; // amps — Kraken X60 (higher continuous rating)
    private static final double HOOD_STATOR_LIMIT    = 40.0; // amps — Kraken X44 (lower continuous rating)

    // ── Shooter constants ──────────────────────────────────────────────────────
    /** Target RPM tolerance to consider the flywheel "at speed" */
    private static final double SHOOTER_RPM_TOLERANCE = 50.0; // RPM

    // Shooter PID + Feedforward (tune these!)
    private static final double SHOOTER_kP = 0.1;
    private static final double SHOOTER_kI = 0.0;
    private static final double SHOOTER_kD = 0.0;
    private static final double SHOOTER_kS = 0.1;  // static friction (V)
    private static final double SHOOTER_kV = 0.12; // volts per RPS

    // ── Hood constants ─────────────────────────────────────────────────────────
    /** Hood angle limits in degrees */
    private static final double HOOD_MIN_DEGREES = 20.0;
    private static final double HOOD_MAX_DEGREES = 60.0;
    // How many output degrees move per one full motor shaft rotation.
    // Formula: 360 / gearRatio  (e.g. 20:1 gearbox → 360 / 20 = 18.0 degrees/rotation)
    // Find your gear ratio in your CAD or gearbox spec sheet.
    private static final double HOOD_DEGREES_PER_ROTATION = 5.0;
    private static final double HOOD_TOLERANCE_DEGREES = 0.5;

    // Hood PID (tune these!)
    private static final double HOOD_kP = 2.0;
    private static final double HOOD_kI = 0.0;
    private static final double HOOD_kD = 0.1;

    // ── Turret rotation constants ──────────────────────────────────────────────
    /** Turret rotation limits in degrees (from center) */
    private static final double TURRET_MIN_DEGREES = -135.0;
    private static final double TURRET_MAX_DEGREES =  135.0;
    // How many output degrees move per one full motor shaft rotation.
    // Formula: 360 / gearRatio  (e.g. 100:1 ring-gear reduction → 360 / 100 = 3.6 degrees/rotation)
    // Find your gear ratio in your CAD or ring gear tooth count: 360 / (ringTeeth / pinionTeeth)
    private static final double TURRET_DEGREES_PER_ROTATION = 10.0;
    private static final double TURRET_TOLERANCE_DEGREES = 1.0;

    // Turret PID + MotionMagic (tune these!)
    private static final double TURRET_kP = 1.5;
    private static final double TURRET_kI = 0.0;
    private static final double TURRET_kD = 0.05;
    private static final double TURRET_kS = 0.1;   // static friction (V) — helps X60 break stiction cleanly
    // MotionMagic smooths turret movement — avoids snapping to setpoint
    // These are in motor shaft rotations per second (RPS), NOT output degrees.
    // To convert a desired output speed to motor RPS:
    //   motorRPS = (desiredDeg/sec) / TURRET_DEGREES_PER_ROTATION
    // Example: want turret to cruise at 360°/s → 360 / 10.0 = 36 RPS at the motor shaft
    private static final double TURRET_MOTION_CRUISE_RPS = 40.0; // motor shaft RPS at peak speed
    private static final double TURRET_MOTION_ACCEL_RPS2 = 80.0; // motor shaft RPS² — time to reach cruise = cruise/accel (e.g. 40/80 = 0.5s)

    // ── Hardware ───────────────────────────────────────────────────────────────
    private final TalonFX shooterLead   = new TalonFX(SHOOTER_LEAD_CAN_ID);
    private final TalonFX shooterFollow = new TalonFX(SHOOTER_FOLLOW_CAN_ID);
    private final TalonFX hoodMotor     = new TalonFX(HOOD_CAN_ID);
    private final TalonFX turretMotor   = new TalonFX(TURRET_CAN_ID); // Kraken X60

    // ── Control requests ───────────────────────────────────────────────────────
    private final VelocityVoltage     shooterControl = new VelocityVoltage(0).withSlot(0);
    private final PositionVoltage     hoodControl    = new PositionVoltage(0).withSlot(0);
    private final MotionMagicVoltage  turretControl  = new MotionMagicVoltage(0).withSlot(0);

    // ── State ──────────────────────────────────────────────────────────────────
    private double targetShooterRPM    = 0.0;
    private double targetHoodDegrees   = HOOD_MIN_DEGREES;
    private double targetTurretDegrees = 0.0;

    // ── Shuffleboard ───────────────────────────────────────────────────────────
    private final ShuffleboardTab tab = Shuffleboard.getTab("Turret");

    // ──────────────────────────────────────────────────────────────────────────
    public Turret() {
        configureShooter();
        configureHood();
        configureTurret();
        setupShuffleboard();
    }

    // ── Configuration ──────────────────────────────────────────────────────────

    private void configureShooter() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.Slot0.kP = SHOOTER_kP;
        cfg.Slot0.kI = SHOOTER_kI;
        cfg.Slot0.kD = SHOOTER_kD;
        cfg.Slot0.kS = SHOOTER_kS;
        cfg.Slot0.kV = SHOOTER_kV;
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        cfg.CurrentLimits.StatorCurrentLimit = SHOOTER_STATOR_LIMIT;
        cfg.CurrentLimits.StatorCurrentLimitEnable = true;

        // Apply identical config to both motors
        shooterLead.getConfigurator().apply(cfg);
        shooterFollow.getConfigurator().apply(cfg);

        // Follower mirrors lead but is mounted inverted — opposeLeaderDirection = true
        shooterFollow.setControl(new Follower(SHOOTER_LEAD_CAN_ID, MotorAlignmentValue.Opposed)); //This one was wonky w/ Claude
    }

    private void configureHood() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.Slot0.kP = HOOD_kP;
        cfg.Slot0.kI = HOOD_kI;
        cfg.Slot0.kD = HOOD_kD;
        cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        // Soft limits are set in motor shaft rotations, so we convert degrees → rotations.
        // Formula: limitRotations = limitDegrees / HOOD_DEGREES_PER_ROTATION
        cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = degreesToRotations(HOOD_MAX_DEGREES, HOOD_DEGREES_PER_ROTATION);
        cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = degreesToRotations(HOOD_MIN_DEGREES, HOOD_DEGREES_PER_ROTATION);
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        cfg.CurrentLimits.StatorCurrentLimit = HOOD_STATOR_LIMIT;
        cfg.CurrentLimits.StatorCurrentLimitEnable = true;
        hoodMotor.getConfigurator().apply(cfg);
    }

    private void configureTurret() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();

        // PID gains
        cfg.Slot0.kP = TURRET_kP;
        cfg.Slot0.kI = TURRET_kI;
        cfg.Slot0.kD = TURRET_kD;
        cfg.Slot0.kS = TURRET_kS;

        // MotionMagic profile — smooth trapezoidal motion to setpoint
        cfg.MotionMagic.MotionMagicCruiseVelocity = TURRET_MOTION_CRUISE_RPS;
        cfg.MotionMagic.MotionMagicAcceleration   = TURRET_MOTION_ACCEL_RPS2;

        // Soft limits
        cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        // Soft limits are set in motor shaft rotations, so we convert degrees → rotations.
        // Formula: limitRotations = limitDegrees / TURRET_DEGREES_PER_ROTATION
        cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = degreesToRotations(TURRET_MAX_DEGREES, TURRET_DEGREES_PER_ROTATION);
        cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = degreesToRotations(TURRET_MIN_DEGREES, TURRET_DEGREES_PER_ROTATION);

        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Kraken X60 current limits
        cfg.CurrentLimits.StatorCurrentLimit = TURRET_STATOR_LIMIT;
        cfg.CurrentLimits.StatorCurrentLimitEnable = true;

        turretMotor.getConfigurator().apply(cfg);
    }

    private void setupShuffleboard() {
        tab.addDouble("Shooter RPM (Target)",   () -> targetShooterRPM);
        tab.addDouble("Shooter Lead RPM",       () -> getShooterRPM());
        tab.addDouble("Shooter Follow RPM",     () -> rpsToRPM(shooterFollow.getVelocity().getValueAsDouble()));
        tab.addBoolean("Shooter At Speed",      () -> isShooterAtSpeed());
        tab.addDouble("Hood Angle (Target)",    () -> targetHoodDegrees);
        tab.addDouble("Hood Angle (Actual)",    () -> getHoodDegrees());
        tab.addBoolean("Hood At Angle",         () -> isHoodAtAngle());
        tab.addDouble("Turret Angle (Target)",  () -> targetTurretDegrees);
        tab.addDouble("Turret Angle (Actual)",  () -> getTurretDegrees());
        tab.addBoolean("Turret At Angle",       () -> isTurretAtAngle());
        tab.addBoolean("Ready To Shoot",        () -> isReadyToShoot());
    }

    // ── Public API ─────────────────────────────────────────────────────────────

    /**
     * Set the shooter flywheel to a target RPM.
     * @param rpm Target RPM (0 to stop)
     */
    public void setShooterRPM(double rpm) {
        targetShooterRPM = rpm;
        if (rpm <= 0) {
            shooterLead.stopMotor();
            // Follower will coast since it follows lead
        } else {
            // Phoenix 6 VelocityVoltage takes RPS (rotations per second), not RPM.
            // Formula: RPS = RPM / 60
            shooterLead.setControl(shooterControl.withVelocity(rpmToRPS(rpm)));
        }
    }

    /** Stop the shooter flywheel. */
    public void stopShooter() {
        setShooterRPM(0);
    }

    /**
     * Set the hood to a target angle in degrees.
     * @param degrees Target angle (clamped to soft limits automatically)
     */
    public void setHoodAngle(double degrees) {
        targetHoodDegrees = clamp(degrees, HOOD_MIN_DEGREES, HOOD_MAX_DEGREES);
        // PositionVoltage takes motor shaft rotations, not degrees.
        // Formula: rotations = degrees / HOOD_DEGREES_PER_ROTATION
        hoodMotor.setControl(hoodControl.withPosition(
            degreesToRotations(targetHoodDegrees, HOOD_DEGREES_PER_ROTATION)));
    }

    /**
     * Set the turret rotation to a target angle in degrees (0 = forward).
     * @param degrees Target angle (clamped to soft limits automatically)
     */
    public void setTurretAngle(double degrees) {
        targetTurretDegrees = clamp(degrees, TURRET_MIN_DEGREES, TURRET_MAX_DEGREES);
        // MotionMagicVoltage takes motor shaft rotations, not degrees.
        // Formula: rotations = degrees / TURRET_DEGREES_PER_ROTATION
        turretMotor.setControl(turretControl.withPosition(
            degreesToRotations(targetTurretDegrees, TURRET_DEGREES_PER_ROTATION)));
    }

    /** Zero the turret encoder at the current position (call when at known home). */
    public void zeroTurret() {
        turretMotor.setPosition(0.0);
    }

    /** Zero the hood encoder at the current position (call when at hard stop).
     *  Seeds the encoder to HOOD_MIN_DEGREES worth of rotations so position
     *  reads correctly from the start. Formula: rotations = minDegrees / degreesPerRotation */
    public void zeroHood() {
        hoodMotor.setPosition(degreesToRotations(HOOD_MIN_DEGREES, HOOD_DEGREES_PER_ROTATION));
    }

    // ── State queries ──────────────────────────────────────────────────────────

    // getVelocity() returns motor shaft RPS — multiply by 60 to get RPM
    public double getShooterRPM() {
        return rpsToRPM(shooterLead.getVelocity().getValueAsDouble());
    }

    // getPosition() returns motor shaft rotations — multiply by degreesPerRotation to get output degrees
    public double getHoodDegrees() {
        return rotationsToDegrees(hoodMotor.getPosition().getValueAsDouble(), HOOD_DEGREES_PER_ROTATION);
    }

    // getPosition() returns motor shaft rotations — multiply by degreesPerRotation to get output degrees
    public double getTurretDegrees() {
        return rotationsToDegrees(turretMotor.getPosition().getValueAsDouble(), TURRET_DEGREES_PER_ROTATION);
    }

    public double getTargetTurretDegrees() {
        return targetTurretDegrees;
    }

    public boolean isShooterAtSpeed() {
        return targetShooterRPM > 0
            && Math.abs(getShooterRPM() - targetShooterRPM) < SHOOTER_RPM_TOLERANCE;
    }

    public boolean isHoodAtAngle() {
        return Math.abs(getHoodDegrees() - targetHoodDegrees) < HOOD_TOLERANCE_DEGREES;
    }

    public boolean isTurretAtAngle() {
        return Math.abs(getTurretDegrees() - targetTurretDegrees) < TURRET_TOLERANCE_DEGREES;
    }

    /** Returns true when all three mechanisms are on target and ready to fire. */
    public boolean isReadyToShoot() {
        return isShooterAtSpeed() && isHoodAtAngle() && isTurretAtAngle();
    }

    // ── Periodic ───────────────────────────────────────────────────────────────

    @Override
    public void periodic() {
        // Telemetry is handled via Shuffleboard lambdas above.
        // Add any periodic safety checks here if needed.
    }

    // ── Utilities ──────────────────────────────────────────────────────────────

    // Phoenix 6 velocity control uses rotations per second (RPS), not RPM.
    // Formula: RPS = RPM / 60
    private static double rpmToRPS(double rpm) { return rpm / 60.0; }

    // Formula: RPM = RPS * 60
    private static double rpsToRPM(double rps) { return rps * 60.0; }

    // TalonFX position is measured in motor shaft rotations, not degrees.
    // Formula: rotations = degrees / degreesPerRotation
    // Example (hood, 20:1 gearbox): 45° / 18.0 = 2.5 motor rotations
    private static double degreesToRotations(double degrees, double degreesPerRotation) {
        return degrees / degreesPerRotation;
    }

    // Inverse of above — convert motor shaft rotations back to output degrees.
    // Formula: degrees = rotations * degreesPerRotation
    private static double rotationsToDegrees(double rotations, double degreesPerRotation) {
        return rotations * degreesPerRotation;
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}