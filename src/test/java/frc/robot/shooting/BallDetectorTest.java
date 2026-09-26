package frc.robot.shooting;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import frc.robot.Constants.Flywheel;

class BallDetectorTest {
    private static final double SPIKE = Flywheel.BALL_CURRENT_A + 10;
    private static final double IDLE = Flywheel.BALL_CURRENT_A - 10;

    @Test
    void spinUpCurrentIsNotABall() {
        BallDetector d = new BallDetector();
        d.update(0.0, 50, 5, 80); // accelerating, huge current
        d.update(0.1, 50, 30, 60);
        assertFalse(d.ballInShooter(0.1));
        assertEquals(0, d.ballsShot());
    }

    @Test
    void countsEachSpikeOnceAfterReachingSpeed() {
        BallDetector d = new BallDetector();
        d.update(0.00, 50, 50, IDLE); // at speed -> armed
        assertTrue(d.armed());
        d.update(0.02, 50, 45, SPIKE); // ball: speed dips, current spikes
        d.update(0.04, 50, 44, SPIKE); // same ball, still in contact
        assertTrue(d.ballInShooter(0.04));
        d.update(0.06, 50, 48, IDLE);
        d.update(0.08, 50, 46, SPIKE); // second ball
        assertEquals(2, d.ballsShot());
    }

    @Test
    void indicatorHoldsBrieflyThenClears() {
        BallDetector d = new BallDetector();
        d.update(0.0, 50, 50, IDLE);
        d.update(0.02, 50, 45, SPIKE);
        d.update(0.04, 50, 49, IDLE);
        assertTrue(d.ballInShooter(0.04));
        assertFalse(d.ballInShooter(0.02 + Flywheel.BALL_HOLD_S + 0.01));
    }

    @Test
    void stoppingTheFlywheelDisarms() {
        BallDetector d = new BallDetector();
        d.update(0.0, 50, 50, IDLE);
        d.update(0.02, 0, 40, SPIKE); // goal 0: braking current is not a ball
        assertFalse(d.armed());
        assertEquals(0, d.ballsShot());
    }
}
