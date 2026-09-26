package frc.robot;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * 2026 REBUILT hub shifts. Pure logic, adapted from the official WPILib example
 * (docs.wpilib.org, "2026 Game Data Details").
 *
 * <p>Both hubs are active in auto, in the 10 s transition shift (match time above 130 s) and in the
 * endgame (30 s and below). In between are four 25 s shifts. About 3 s after auto, the field sends a
 * game-specific message 'R' or 'B': the alliance whose hub goes inactive FIRST (shifts 1 and 3).
 */
public final class HubSchedule {
    private HubSchedule() {}

    /** Match-time boundaries (seconds remaining in teleop) where hub state can change. */
    private static final double[] BOUNDARIES = {130, 105, 80, 55, 30};

    /**
     * @param active            our hub is scoring right now
     * @param secondsUntilChange time until our hub flips state, or +infinity if it won't flip again
     * @param known             false when we can't tell yet (no alliance or no game data); then active = true
     */
    public record Status(boolean active, double secondsUntilChange, boolean known) {
        public String label() {
            if (!known) return "ACTIVE (no game data)";
            String state = active ? "ACTIVE" : "INACTIVE";
            return Double.isInfinite(secondsUntilChange) ? state : String.format("%s %.0fs", state, secondsUntilChange);
        }
    }

    public static Status status(Optional<Alliance> alliance, boolean autonomous, boolean teleop,
            double matchTimeS, String gameData) {
        if (autonomous) return new Status(true, Double.POSITIVE_INFINITY, true);
        if (alliance.isEmpty() || !teleop || gameData == null || gameData.isEmpty()) {
            return new Status(true, Double.POSITIVE_INFINITY, false);
        }
        boolean redInactiveFirst;
        switch (gameData.charAt(0)) {
            case 'R' -> redInactiveFirst = true;
            case 'B' -> redInactiveFirst = false;
            default -> {
                return new Status(true, Double.POSITIVE_INFINITY, false);
            }
        }
        boolean shift1Active = alliance.get() == Alliance.Red ? !redInactiveFirst : redInactiveFirst;
        boolean active = activeAt(matchTimeS, shift1Active);

        // Walk the remaining boundaries to find when our state next changes.
        double until = Double.POSITIVE_INFINITY;
        for (double b : BOUNDARIES) {
            if (b < matchTimeS && activeAt(b - 0.001, shift1Active) != active) {
                until = matchTimeS - b;
                break;
            }
        }
        return new Status(active, until, true);
    }

    private static boolean activeAt(double matchTimeS, boolean shift1Active) {
        if (matchTimeS > 130) return true;
        if (matchTimeS > 105) return shift1Active;
        if (matchTimeS > 80) return !shift1Active;
        if (matchTimeS > 55) return shift1Active;
        if (matchTimeS > 30) return !shift1Active;
        return true;
    }
}
