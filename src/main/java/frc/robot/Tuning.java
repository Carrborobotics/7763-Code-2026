package frc.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Numbers that can be changed live from the dashboard ("Tuning/..." on SmartDashboard) without a
 * redeploy. Defaults come from {@link Constants}; unit tests always see the defaults because only
 * Robot calls {@link #update()}.
 *
 * <p>Workflow: tune on the dashboard, press "Tuning/Print Values", then copy the printed lines back into
 * Constants so the values survive a reboot.
 */
public final class Tuning {
    private Tuning() {}

    public static final class Value {
        public final String key;
        public final double defaultValue;
        private double value;

        private Value(String key, double defaultValue) {
            this.key = key;
            this.defaultValue = defaultValue;
            this.value = defaultValue;
        }

        public double get() {
            return value;
        }
    }

    private static final List<Value> ALL = new ArrayList<>();
    private static final String PRINT = "Tuning/Print Values";

    private static Value add(String key, double defaultValue) {
        Value v = new Value(key, defaultValue);
        ALL.add(v);
        return v;
    }

    // ---- shot table (distances stay fixed; speeds and times of flight are tunable) ----
    public static final Value[] SHOT_RPS = new Value[Constants.Shooting.SHOT_TABLE.length];
    public static final Value[] SHOT_TOF_S = new Value[Constants.Shooting.SHOT_TABLE.length];

    static {
        double[][] t = Constants.Shooting.SHOT_TABLE;
        for (int i = 0; i < t.length; i++) {
            SHOT_RPS[i] = add("Shot Table/" + t[i][0] + " m rps", t[i][1]);
            SHOT_TOF_S[i] = add("Shot Table/" + t[i][0] + " m tof s", t[i][2]);
        }
    }

    public static final Value HOOD_UNITS_PER_METER = add("Hood/Units Per Meter", Constants.Hood.UNITS_PER_METER);

    // ---- ball detection ----
    public static final Value BALL_CURRENT_A = add("Flywheel/Ball Current A", Constants.Flywheel.BALL_CURRENT_A);
    public static final Value AT_SPEED_TOLERANCE_RPS =
            add("Flywheel/At Speed Tolerance rps", Constants.Flywheel.AT_SPEED_TOLERANCE_RPS);

    // ---- feed gating ----
    public static final Value TURRET_ON_TARGET_DEG = add("Feed/Turret On Target deg", Constants.Feed.TURRET_ON_TARGET_DEG);
    public static final Value FEED_GATE_TIMEOUT_S = add("Feed/Gate Timeout s", Constants.Feed.GATE_TIMEOUT_S);
    public static final Value FLOOR_UNJAM_S = add("Feed/Floor Unjam s", Constants.Floor.UNJAM_S);
    public static final Value FLOOR_UNJAM_DUTY = add("Feed/Floor Unjam duty", Constants.Floor.UNJAM_DUTY);

    // ---- reverse-all ----
    public static final Value REVERSE_INTAKE = add("Reverse/Intake duty", Constants.Reverse.INTAKE_DUTY);
    public static final Value REVERSE_FLOOR = add("Reverse/Floor duty", Constants.Reverse.FLOOR_DUTY);
    public static final Value REVERSE_KICKER = add("Reverse/Kicker duty", Constants.Reverse.KICKER_DUTY);
    public static final Value REVERSE_FLYWHEEL_RPS = add("Reverse/Flywheel rps", Constants.Reverse.FLYWHEEL_RPS);

    // ---- tilt compensation geometry ----
    public static final Value HOOD_UNITS_PER_DEGREE =
            add("Tilt/Hood Units Per Degree", Constants.Shooting.HOOD_UNITS_PER_DEGREE);
    public static final Value LAUNCH_DEG_AT_MIN_HOOD =
            add("Tilt/Launch Deg At Min Hood", Constants.Shooting.LAUNCH_DEG_AT_MIN_HOOD);

    /** Put every value on the dashboard (keeps anything already there, e.g. from a previous boot). */
    public static void publish() {
        for (Value v : ALL) SmartDashboard.setDefaultNumber("Tuning/" + v.key, v.defaultValue);
        SmartDashboard.putBoolean(PRINT, false);
    }

    /** Read the dashboard. Called once per loop by Robot. */
    public static void update() {
        for (Value v : ALL) v.value = SmartDashboard.getNumber("Tuning/" + v.key, v.defaultValue);
        if (SmartDashboard.getBoolean(PRINT, false)) {
            SmartDashboard.putBoolean(PRINT, false);
            System.out.println("==== Tuning values (copy into Constants) ====");
            for (Value v : ALL) {
                String changed = v.value != v.defaultValue ? "   <-- changed from " + v.defaultValue : "";
                System.out.println(v.key + " = " + v.value + changed);
            }
        }
    }
}
