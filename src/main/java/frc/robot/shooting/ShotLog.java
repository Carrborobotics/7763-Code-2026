package frc.robot.shooting;

import java.util.ArrayList;
import java.util.List;

/**
 * Records every detected shot so the shot table can be rebuilt from real data.
 *
 * <p>Balls are fired in volleys, so results are marked per volley: after a volley the driver (or a
 * scout) presses "Mark Made" or "Mark Missed" on the dashboard and every unmarked shot gets that result.
 * Shots left unmarked are written as "unknown" when the next volley starts or the robot disables.
 * Pure logic: Robot writes the finished CSV lines to a file.
 */
public final class ShotLog {
    public static final String CSV_HEADER =
            "id,time_s,match_time_s,mode,distance_m,goal_rps,measured_rps,hood_units,turret_error_deg,"
                    + "robot_speed_mps,tilt_deg,result";

    /** One detected ball. */
    public record Shot(int id, double timeS, double matchTimeS, String mode, double distanceM, double goalRps,
            double measuredRps, double hoodUnits, double turretErrorDeg, double robotSpeedMps, double tiltDeg) {

        String toCsv(String result) {
            return String.format("%d,%.3f,%.1f,%s,%.3f,%.2f,%.2f,%.1f,%.2f,%.2f,%.1f,%s", id, timeS, matchTimeS,
                    mode, distanceM, goalRps, measuredRps, hoodUnits, turretErrorDeg, robotSpeedMps, tiltDeg, result);
        }
    }

    /** A new volley starts if this long passes between shots; the previous one gets written as unknown. */
    public static final double VOLLEY_GAP_S = 5.0;

    private final List<Shot> unmarked = new ArrayList<>();
    private int nextId = 1;
    private int made = 0;
    private int missed = 0;

    /**
     * Record a shot. Returns CSV lines that are now final (the previous volley, if it went unmarked).
     * Pass the shot's fields except the id, which is assigned here.
     */
    public List<String> record(double timeS, double matchTimeS, String mode, double distanceM, double goalRps,
            double measuredRps, double hoodUnits, double turretErrorDeg, double robotSpeedMps, double tiltDeg) {
        List<String> finished = new ArrayList<>();
        if (!unmarked.isEmpty() && timeS - unmarked.get(unmarked.size() - 1).timeS() > VOLLEY_GAP_S) {
            finished.addAll(flush("unknown"));
        }
        unmarked.add(new Shot(nextId++, timeS, matchTimeS, mode, distanceM, goalRps, measuredRps, hoodUnits,
                turretErrorDeg, robotSpeedMps, tiltDeg));
        return finished;
    }

    /** Mark every unmarked shot. Returns their CSV lines. */
    public List<String> mark(boolean wasMade) {
        List<String> lines = flush(wasMade ? "made" : "missed");
        if (wasMade) made += lines.size();
        else missed += lines.size();
        return lines;
    }

    /** Write unmarked shots out as unknown (e.g. on disable). */
    public List<String> flushUnknown() {
        return flush("unknown");
    }

    private List<String> flush(String result) {
        List<String> lines = new ArrayList<>();
        for (Shot s : unmarked) lines.add(s.toCsv(result));
        unmarked.clear();
        return lines;
    }

    public int unmarkedCount() {
        return unmarked.size();
    }

    public int madeCount() {
        return made;
    }

    public int missedCount() {
        return missed;
    }
}
