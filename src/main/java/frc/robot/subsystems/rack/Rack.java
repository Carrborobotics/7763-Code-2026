package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RackAndPinionIntake extends SubsystemBase {

    // ── Hardware ──────────────────────────────────────────────────────────────
    private final TalonFX m_leader;
    private final TalonFX m_follower;

    // ── Control request (reused every loop to avoid GC pressure) ─────────────
    private final MotionMagicVoltage m_mmRequest = new MotionMagicVoltage(0)
            .withSlot(0)
            .withEnableFOC(false);   // set true once FOC license is setup

    // ── Setpoints (rotations at the motor — adjust to your mechanism) ─────────
    public static final class Setpoints {
        public static final double RETRACTED  = 0.0;   // TODO: tune
        public static final double DEPLOYED   = 10.0;  // TODO: tune
        public static final double PARTIAL    = 5.0;   // TODO: tune (optional mid-point)
    }

    // ── Tolerances ────────────────────────────────────────────────────────────
    private static final double POSITION_TOLERANCE_ROTATIONS = 0.2;

    // ── Constructor ───────────────────────────────────────────────────────────
    public RackAndPinionIntake(int leaderCanId, int followerCanId) {
        m_leader   = new TalonFX(leaderCanId,   "rio"); // change bus name if on CANivore
        m_follower = new TalonFX(followerCanId, "rio");

        configureMotors();
    }

    // ── Configuration ─────────────────────────────────────────────────────────
    private void configureMotors() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();

        // --- Neutral mode ---
        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // --- Soft limits (protect your mechanism) ---
        cfg.SoftwareLimitSwitch.ForwardSoftLimitEnable    = true;
        cfg.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 10.5; // TODO: tune
        cfg.SoftwareLimitSwitch.ReverseSoftLimitEnable    = true;
        cfg.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -0.5; // TODO: tune

        // --- Slot 0 PID gains ---
        // Start conservative; tune kP first, then kD, then kS/kV/kA
        cfg.Slot0.kP = 2.4;   // TODO: tune — output per rotation of error
        cfg.Slot0.kI = 0.0;
        cfg.Slot0.kD = 0.1;   // TODO: tune
        cfg.Slot0.kS = 0.25;  // TODO: tune — static friction (volts)
        cfg.Slot0.kV = 0.12;  // TODO: tune — volts per RPS
        cfg.Slot0.kA = 0.01;  // TODO: tune — volts per RPS²

        // --- MotionMagic cruise / acceleration ---
        cfg.MotionMagic.MotionMagicCruiseVelocity = 80;   // RPS   TODO: tune
        cfg.MotionMagic.MotionMagicAcceleration   = 160;  // RPS/s TODO: tune
        cfg.MotionMagic.MotionMagicJerk            = 1600; // RPS/s² — smooths the profile

        // Apply config to leader
        m_leader.getConfigurator().apply(cfg);

        // Follower mirrors the leader (set opposeLeaderDirection=true if it's
        // physically mirrored and needs to spin the opposite direction)
        m_follower.setControl(new Follower(m_leader.getDeviceID(), /* opposeLeaderDirection= */ false));
    }

    // ── Public API ────────────────────────────────────────────────────────────

    /** Move to an arbitrary position (motor rotations). */
    public void setPosition(double rotations) {
        m_leader.setControl(m_mmRequest.withPosition(rotations));
    }

    /** Convenience wrappers for named setpoints. */
    public void retract()  { setPosition(Setpoints.RETRACTED); }
    public void deploy()   { setPosition(Setpoints.DEPLOYED);  }
    public void partial()  { setPosition(Setpoints.PARTIAL);   }

    /** Stop the motor and let brake mode hold position. */
    public void stop() {
        m_leader.stopMotor();
    }

    /** Seed the encoder — call this once at match start if you have a known home. */
    public void resetPosition(double knownRotations) {
        m_leader.setPosition(knownRotations);
    }

    // ── Telemetry helpers ─────────────────────────────────────────────────────

    public double getPosition() {
        return m_leader.getPosition().getValueAsDouble();
    }

    public double getVelocity() {
        return m_leader.getVelocity().getValueAsDouble();
    }

    public boolean atSetpoint(double targetRotations) {
        return Math.abs(getPosition() - targetRotations) < POSITION_TOLERANCE_ROTATIONS;
    }

    // ── Periodic ──────────────────────────────────────────────────────────────
    @Override
    public void periodic() {
        // Add SmartDashboard/logging here as needed, e.g.:
        // SmartDashboard.putNumber("Intake/Position", getPosition());
        // SmartDashboard.putNumber("Intake/Velocity", getVelocity());
    }
}