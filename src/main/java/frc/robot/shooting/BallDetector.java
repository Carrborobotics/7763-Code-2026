package frc.robot.shooting;

import frc.robot.Constants.Flywheel;
import frc.robot.Tuning;

/**
 * Detects balls passing through the flywheel from motor current. Pure logic: Robot feeds it the
 * flywheel goal, measured speed and stator current each loop.
 *
 * <p>Spinning up also draws a lot of current, so detection only "arms" once the flywheel has reached
 * its goal speed, and stays armed (even through the speed dip a ball causes) until the goal goes to 0.
 */
public final class BallDetector {
    private boolean armed = false;
    private boolean ballLastLoop = false;
    private double lastBallS = Double.NEGATIVE_INFINITY;
    private int ballsShot = 0;

    public void update(double nowS, double goalRps, double measuredRps, double statorAmps) {
        if (goalRps <= 0) {
            armed = false;
        } else if (Math.abs(measuredRps - goalRps) < Tuning.AT_SPEED_TOLERANCE_RPS.get()) {
            armed = true;
        }

        boolean ballNow = armed && statorAmps > Tuning.BALL_CURRENT_A.get();
        if (ballNow) {
            if (!ballLastLoop) ballsShot++;
            lastBallS = nowS;
        }
        ballLastLoop = ballNow;
    }

    /** True while a ball is in the flywheel, held briefly so a dashboard can show it. */
    public boolean ballInShooter(double nowS) {
        return nowS - lastBallS <= Flywheel.BALL_HOLD_S;
    }

    public int ballsShot() {
        return ballsShot;
    }

    public boolean armed() {
        return armed;
    }

    public void resetCount() {
        ballsShot = 0;
    }
}
