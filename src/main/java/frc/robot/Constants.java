package frc.robot;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * Every tunable number on the robot lives here. Nothing else should hard-code a CAN ID, gear ratio,
 * gain, speed or field coordinate.
 *
 * <p>Unit conventions (the suffix on each name says which one applies):
 * <ul>
 *   <li>distances in meters, angles in degrees unless suffixed {@code Rad} / {@code Rot}
 *   <li>{@code Rot} = mechanism rotations as the TalonFX reports them (after SensorToMechanismRatio)
 *   <li>{@code Rps} = rotations per second
 *   <li>{@code Duty} = percent output, -1..1
 * </ul>
 */
public final class Constants {
    private Constants() {}

    public static final double LOOP_PERIOD_S = 0.02;

    public static final class CAN {
        public static final CANBus RIO = new CANBus("rio");
        public static final CANBus DRIVETRAIN = new CANBus("Drivetrain"); // CANivore

        // Drivetrain CANivore
        public static final int PIGEON = 10;
        public static final int TURRET = 21;

        // rio bus
        public static final int FLYWHEEL_LEFT = 22;
        public static final int FLYWHEEL_RIGHT = 23;
        public static final int KICKER = 24;
        public static final int HOOD = 25;
        public static final int FLOOR = 30;
        public static final int RACK_A = 41;
        public static final int INTAKE = 42;
        public static final int RACK_B = 43;
    }

    public static final class Driver {
        public static final int PORT = 0;
        public static final double DEADBAND = 0.12;
        /** POV up/down changes the flywheel speed offset by this much. */
        public static final double FLYWHEEL_OFFSET_STEP_RPS = 2.0;
        /** POV left/right changes the turret aim offset by this much. */
        public static final double TURRET_OFFSET_STEP_DEG = 2.0;
    }

    public static final class Drive {
        /**
         * false = our own swerve code (proven at competition). true = CTRE's generated-style swerve library
         * (250 Hz odometry, time-synced). Verify on blocks before driving: every wheel points forward and
         * pushing the stick forward drives forward.
         */
        public static final boolean USE_CTRE_SWERVE = false;

        public static final double MAX_SPEED_MPS = 4.5;
        public static final double MAX_OMEGA_RAD_PER_S = 10.0;
        /** Speed limits while the driver is holding shoot (shoot-on-the-move). */
        public static final double SHOOTING_MAX_SPEED_MPS = 0.5;
        public static final double SHOOTING_MAX_OMEGA_RAD_PER_S = 2.0;

        // WCP SwerveX Flipped, Kraken X60, X3 10-tooth
        public static final double WHEEL_DIAMETER_M = Units.inchesToMeters(4.0);
        public static final double WHEEL_CIRCUMFERENCE_M = WHEEL_DIAMETER_M * Math.PI;
        public static final double DRIVE_RATIO = 5.51;
        public static final double STEER_RATIO = 12.1;

        public static final double TRACK_WIDTH_M = Units.inchesToMeters(25);
        public static final double WHEEL_BASE_M = Units.inchesToMeters(25);
        public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
                new Translation2d(WHEEL_BASE_M / 2, TRACK_WIDTH_M / 2), // FL
                new Translation2d(WHEEL_BASE_M / 2, -TRACK_WIDTH_M / 2), // FR
                new Translation2d(-WHEEL_BASE_M / 2, TRACK_WIDTH_M / 2), // BL
                new Translation2d(-WHEEL_BASE_M / 2, -TRACK_WIDTH_M / 2)); // BR

        public static final double STEER_KP = 50.0;
        public static final double DRIVE_KP = 0.12;
        public static final double DRIVE_KS = 0.32;
        public static final double DRIVE_KV = 1.51;
        public static final double DRIVE_KA = 0.27;
        public static final double DRIVE_SUPPLY_LIMIT_A = 60;
        public static final double OPEN_LOOP_RAMP_S = 0.25;

        /**
         * The same drive gains converted to motor-rotor units for the CTRE library, which runs the drive
         * loop on rotor rps (1 m/s = DRIVE_RATIO / WHEEL_CIRCUMFERENCE_M rotor rps).
         */
        public static final double ROTOR_RPS_PER_MPS = DRIVE_RATIO / WHEEL_CIRCUMFERENCE_M;
        public static final double CTRE_DRIVE_KP = DRIVE_KP / DRIVE_RATIO;
        public static final double CTRE_DRIVE_KS = DRIVE_KS;
        public static final double CTRE_DRIVE_KV = DRIVE_KV / ROTOR_RPS_PER_MPS;
        public static final double CTRE_DRIVE_KA = DRIVE_KA / ROTOR_RPS_PER_MPS;
        public static final double ODOMETRY_HZ = 250;

        /** PathPlanner holonomic controller gains. */
        public static final double AUTO_TRANSLATION_KP = 5.0;
        public static final double AUTO_ROTATION_KP = 5.0;

        /** Module IDs and CANcoder offsets (rotations). Order must match KINEMATICS. */
        public record Module(String name, int driveId, int steerId, int cancoderId, Rotation2d offset) {}

        public static final Module[] MODULES = {
            new Module("FL", 1, 2, 15, Rotation2d.fromRotations(0.285400390625)),
            new Module("FR", 3, 4, 12, Rotation2d.fromRotations(0.608642578125)),
            new Module("BL", 7, 8, 13, Rotation2d.fromRotations(0.748046875)),
            new Module("BR", 5, 6, 14, Rotation2d.fromRotations(0.720703125)),
        };
    }

    /** Wait-for-ready feeding: hold the floor + kicker until the shot is lined up. */
    public static final class Feed {
        /** Boot default for the "Driver/Wait For Ready" dashboard toggle. */
        public static final boolean WAIT_FOR_READY = true;
        public static final double TURRET_ON_TARGET_DEG = 3.0;
        /** Feed anyway after this long, so a bad reading can never block a shot. */
        public static final double GATE_TIMEOUT_S = 0.75;
    }

    public static final class Hub {
        /** Boot default for "Driver/Hold Fire When Hub Off": don't feed into our hub while it's inactive. */
        public static final boolean HOLD_FIRE_WHEN_INACTIVE = false;
        /** Rumble the controller this many seconds before our hub turns on. */
        public static final double WARN_BEFORE_ACTIVE_S = 3.0;
    }

    public static final class Health {
        public static final double LOW_BATTERY_V = 11.5;
        public static final double CHECK_PERIOD_S = 0.5;
    }

    public static final class Rumble {
        public static final double BALL_SHOT_STRENGTH = 0.5;
        public static final double BALL_SHOT_S = 0.12;
        public static final double READY_STRENGTH = 0.3;
        public static final double READY_S = 0.2;
        public static final double HUB_WARNING_STRENGTH = 0.8;
        public static final double HUB_WARNING_S = 0.4;
    }

    public static final class Intake {
        public static final double IN_DUTY = 0.9;
        public static final double OUT_DUTY = -0.2;
    }

    public static final class Floor {
        public static final double FEED_DUTY = 0.5;
    }

    public static final class Kicker {
        public static final double FEED_DUTY = 0.5;
    }

    /** Reverse-everything button: every roller in the superstructure runs backwards. UNTUNED guesses. */
    public static final class Reverse {
        public static final double INTAKE_DUTY = -0.3;
        public static final double FLOOR_DUTY = -0.3;
        public static final double KICKER_DUTY = -0.3;
        public static final double FLYWHEEL_RPS = -10.0;
    }

    public static final class Rack {
        // Legacy values were written as "degrees" of motor shaft: -32000 deg = -88.9 motor rotations.
        public static final double EXTENDED_ROT = -32000.0 / 360.0;
        public static final double RETRACTED_ROT = 0.0;
        public static final double KP = 0.8;
        /** Agitate cycles through these fractions of EXTENDED (repeating while held), AGITATE_STEP_S each. */
        public static final double[] AGITATE_FRACTIONS = {1.0, 0.5, 0.75, 0.5};
        public static final double AGITATE_STEP_S = 0.4;
        /** How long the auto "Agitate" named command runs before restoring the rack. */
        public static final double AGITATE_AUTO_S = 2.0;
    }

    public static final class Flywheel {
        public static final double KP = 0.1;
        public static final double KS = 0.25;
        public static final double KV = 0.12;
        public static final double AT_SPEED_TOLERANCE_RPS = 3.0;

        /**
         * Ball detection: once the flywheel has reached speed, a ball squeezing through shows up as a
         * stator-current spike (average of both motors). UNTUNED: watch Flywheel/StatorAmps while shooting
         * and set the threshold between the idle-at-speed current and the spike.
         */
        public static final double BALL_CURRENT_A = 20.0;
        /** Keep "ball in shooter" lit this long after the spike so a dashboard can show it. */
        public static final double BALL_HOLD_S = 0.15;
    }

    public static final class Hood {
        /**
         * The hood is commanded in legacy "hood units" (NOT degrees). Motor position = -units / 360
         * mechanism rotations. 0 = where the hood was at boot / after homing (fully down).
         */
        public static final double MIN_UNITS = 100.0;
        public static final double MAX_UNITS = 380.0;
        /** Tuned shot map: hood units = distance (m) * this, clamped to [MIN, MAX]. */
        public static final double UNITS_PER_METER = 60.0;
        public static final double SENSOR_TO_MECHANISM = 45.0;
        public static final double KP = 300.0;
        public static final double KS = 0.1;
        public static final double KV = 0.1;
        public static final double STATOR_LIMIT_A = 40.0;

        /** Homing (start button): drive toward the down hard stop until current spikes, then zero. */
        public static final double HOMING_VOLTS = 2.0;
        public static final double HOMING_CURRENT_A = 5.0;
        public static final double HOMING_IGNORE_S = 0.25; // ignore start-up current spike
        public static final double HOMING_DEBOUNCE_S = 0.2;
        public static final double HOMING_TIMEOUT_S = 3.0;
    }

    public static final class Turret {
        /** Turret angles are robot-relative, CCW positive, 0 = robot forward. Motor sign is flipped. */
        public static final double SENSOR_TO_MECHANISM = 26.0;
        public static final double KP = 90.0;
        public static final double KS = 5.0;
        public static final double KV = 10.0;
        public static final double PEAK_VOLTS = 6.0;
        /**
         * The turret can't spin forever. Commands are wrapped into [WRAP_CENTER-180, WRAP_CENTER+180],
         * i.e. about [-95, 265] degrees.
         */
        public static final double WRAP_CENTER_DEG = 85.0;
        /** Turret pivot relative to robot center, in polar form (robot frame). */
        public static final double PIVOT_RADIUS_M = 0.286;
        public static final double PIVOT_ANGLE_RAD = 0.854;
    }

    public static final class Shooting {
        /** Shoot-on-the-move lead passes: 0 = stationary aim, 1 = single pass, 3 = converged. */
        public static final int SOTM_ITERATIONS = 1;

        /** distance (m), flywheel (rps), time of flight (s). Must be sorted by distance. */
        public static final double[][] SHOT_TABLE = {
            {1.0, 39.0, 0.60},
            {1.5, 42.5, 0.65},
            {2.0, 45.0, 0.70},
            {3.0, 52.0, 0.75},
            {4.0, 56.0, 0.90},
            {5.0, 58.0, 1.00},
            {6.0, 65.0, 1.10},
            {8.0, 80.0, 1.20},
        };

        /** Drop the hood this close to a trench so we fit under it. */
        public static final double OUR_TRENCH_CLEARANCE_M = Units.inchesToMeters(12);
        public static final double FAR_TRENCH_CLEARANCE_M = Units.inchesToMeters(30);

        /**
         * "Shoot on the bump" tilt compensation (TiltCompensation). This is the boot default; the
         * "Driver/Tilt Compensation" toggle on the dashboard switches it live.
         */
        public static final boolean TILT_COMPENSATION = true;
        /** Only compensate when roll/pitch exceeds this, so gyro noise on flat carpet does nothing. */
        public static final double TILT_MIN_DEG = 3.0;

        // ---- Hood geometry for tilt compensation. ESTIMATES: measure and replace. ----
        /**
         * Hood units per degree of launch angle. Estimate from gearing: the hood is 715.5:1 from the motor
         * and the TalonFX ratio is 45:1, so 1 hood unit = 1/360 of a TalonFX "mechanism" rotation
         * = 1/15.9 of a hood degree. That makes the 100..380 unit range about 18 degrees of hood travel.
         */
        public static final double HOOD_UNITS_PER_DEGREE = 715.5 / 45.0;
        /** Launch angle above horizontal with the hood at MIN_UNITS (steepest, close shots). GUESS. */
        public static final double LAUNCH_DEG_AT_MIN_HOOD = 65.0;
        /** true: more hood units = flatter shot (far shots use more units). GUESS from the shot map. */
        public static final boolean MORE_HOOD_UNITS_IS_FLATTER = true;
        /** Safety clamps so a wrong estimate can only nudge the aim, never fling it. */
        public static final double MAX_TILT_TURRET_CORRECTION_DEG = 10.0;
        public static final double MAX_TILT_HOOD_CORRECTION_UNITS = 80.0;
    }

    /** Simulation-only mechanism models (rough guesses; they only need to behave plausibly). */
    public static final class Sim {
        public static final double TURRET_MOI = 0.05;
        public static final double HOOD_MOI = 0.005;
        public static final double FLYWHEEL_MOI = 0.003;
        public static final double ROLLER_MOI = 0.0005;
        public static final double RACK_MOI = 0.0005;
        /** Fraction of flywheel speed lost when a simulated ball goes through. */
        public static final double BALL_SPEED_LOSS = 0.12;
        public static final double BALL_INTERVAL_S = 0.3;
    }

    /** SysId characterization (test mode only). */
    public static final class SysId {
        public static final double DRIVE_RAMP_V_PER_S = 1.0;
        public static final double DRIVE_STEP_V = 4.0;
        public static final double DRIVE_TIMEOUT_S = 4.0;
        public static final double FLYWHEEL_RAMP_V_PER_S = 1.0;
        public static final double FLYWHEEL_STEP_V = 7.0;
        public static final double FLYWHEEL_TIMEOUT_S = 10.0;
    }

    /** Field geometry, blue-alliance origin. We build on the AndyMark field. */
    public static final class Field {
        public static final AprilTagFieldLayout TAGS =
                AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

        public static final double LENGTH_M = Units.inchesToMeters(650.12);
        public static final double WIDTH_M = Units.inchesToMeters(316.64);
        public static final double TRENCH_X_M = Units.inchesToMeters(181.56);
        public static final double FAR_TRENCH_X_M = LENGTH_M - TRENCH_X_M;
        public static final double MID_Y_M = WIDTH_M / 2;

        public static final Translation2d HUB = new Translation2d(TRENCH_X_M, MID_Y_M);
        /** Passing targets used when we are past our trench line. */
        public static final Translation2d PASS_LOW =
                new Translation2d(Units.inchesToMeters(110), Units.inchesToMeters(80));
        public static final Translation2d PASS_HIGH =
                new Translation2d(Units.inchesToMeters(110), WIDTH_M - Units.inchesToMeters(80));
    }

    public static final class Vision {
        public static final String CAM_FRONT = "Arducam_OV9281_USB_Camera";
        public static final String CAM_SIDE = "Arducam_OV9782_USB_Camera";

        public static final Transform3d ROBOT_TO_CAM_FRONT = new Transform3d(
                new Translation3d(Units.inchesToMeters(13), Units.inchesToMeters(-2.5), Units.inchesToMeters(12)),
                new Rotation3d(0, Units.degreesToRadians(25), 0));
        /*
         * CHECK THIS ONE: y = +11.5 in puts the camera on the robot's LEFT side, but yaw 270 deg points it
         * to the RIGHT (through the robot). If it is mounted on the right side, y should be negative;
         * if it faces left, yaw should be 90. A wrong transform makes every estimate land off the field
         * or in the air, and Vision rejects it ("Vision/Side Status" on the dashboard says which).
         */
        public static final Transform3d ROBOT_TO_CAM_SIDE = new Transform3d(
                new Translation3d(Units.inchesToMeters(11.25), Units.inchesToMeters(11.5), Units.inchesToMeters(12.25)),
                new Rotation3d(0, Units.degreesToRadians(25), Units.degreesToRadians(270)));

        /** Base std devs (x m, y m, theta rad), scaled up by 1 + d^2/30 with average tag distance d. */
        public static final double[] MULTI_TAG_STD = {0.5, 0.5, 1.0};
        public static final double[] SINGLE_TAG_STD = {1.0, 1.0, 2.0};
        public static final double MAX_SINGLE_TAG_DIST_M = 4.0;
        public static final double MAX_AMBIGUITY = 0.2;
        public static final double MAX_Z_ERROR_M = 0.5;
    }
}
