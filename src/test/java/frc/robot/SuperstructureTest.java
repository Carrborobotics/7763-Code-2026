package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import frc.robot.Constants.Field;
import frc.robot.Constants.Floor;
import frc.robot.Constants.Hood;
import frc.robot.Constants.Intake;
import frc.robot.Constants.Kicker;
import frc.robot.Constants.Rack;
import frc.robot.Constants.Reverse;
import frc.robot.Constants.Feed;
import frc.robot.Superstructure.FeedMode;
import frc.robot.Superstructure.IntakeMode;
import frc.robot.Superstructure.Measured;
import frc.robot.Superstructure.Setpoints;
import frc.robot.shooting.ShotCalculator.Shot;

class SuperstructureTest {
    private static final double EPS = 1e-9;
    private static final Shot SHOT = new Shot(Field.HUB, 3.0, 10.0, 180.0, 52.0, false, 0, 0);

    private Superstructure ss;

    @BeforeEach
    void setup() {
        ss = new Superstructure();
    }

    /** Flywheel at speed and turret on target, so feeding is never held back. */
    private static final Measured READY = new Measured(0, true, 0);
    private static final Measured NOT_READY = new Measured(0, false, 10);
    /** First time after Shoot is pressed (at t=0) that the floor runs forwards. */
    private static final double AFTER_UNJAM = Floor.UNJAM_S + 0.01;

    private Setpoints at(double t) {
        return ss.update(t, SHOT, READY);
    }

    private Setpoints hoodAmps(double t, double amps) {
        return ss.update(t, SHOT, new Measured(amps, true, 0));
    }

    @Test
    void idleAimsButRunsNothing() {
        Setpoints sp = at(0);
        assertEquals(10.0, sp.turretDeg(), EPS);
        assertEquals(180.0, sp.hoodUnits(), EPS);
        assertEquals(0, sp.flywheelRps());
        assertEquals(0, sp.kickerDuty());
        assertEquals(0, sp.floorDuty());
        assertEquals(0, sp.intakeDuty());
        assertEquals(Rack.RETRACTED_ROT, sp.rackRot(), EPS);
        assertEquals(0, sp.hoodHomingVolts());
    }

    @Test
    void shootSpinsFlywheelAndFeeds() {
        ss.setFeed(FeedMode.SHOOT);
        Setpoints sp = at(0);
        assertEquals(52.0, sp.flywheelRps(), EPS);
        assertEquals(Kicker.FEED_DUTY, sp.kickerDuty(), EPS);
        assertEquals(Floor.FEED_DUTY, at(AFTER_UNJAM).floorDuty(), EPS);
    }

    @Test
    void floorRunsBackwardsBrieflyAtTheStartOfEveryShot() {
        ss.setFeed(FeedMode.SHOOT);
        assertEquals(Floor.UNJAM_DUTY, at(0).floorDuty(), EPS);
        assertEquals(Floor.UNJAM_DUTY, at(Floor.UNJAM_S - 0.01).floorDuty(), EPS);
        assertEquals(Floor.FEED_DUTY, at(AFTER_UNJAM).floorDuty(), EPS);

        ss.setFeed(FeedMode.OFF);
        assertEquals(0, at(2.0).floorDuty());
        ss.setFeed(FeedMode.SHOOT);
        assertEquals(Floor.UNJAM_DUTY, at(3.0).floorDuty(), EPS, "a new press unjams again");
        assertEquals(Floor.UNJAM_DUTY, ss.update(3.1, SHOT, NOT_READY).floorDuty(), EPS, "even while waiting for ready");
    }

    @Test
    void driverOffsetsApply() {
        ss.setFeed(FeedMode.SHOOT);
        ss.adjustFlywheelOffset(2);
        ss.adjustFlywheelOffset(2);
        ss.adjustTurretOffset(-2);
        Setpoints sp = at(0);
        assertEquals(56.0, sp.flywheelRps(), EPS);
        assertEquals(8.0, sp.turretDeg(), EPS);
    }

    @Test
    void reverseRunsEveryRollerBackwardsAndOverridesShootAndIntake() {
        ss.setFeed(FeedMode.SHOOT);
        ss.setIntake(IntakeMode.IN);
        ss.setReverse(true);
        Setpoints sp = at(0);
        assertEquals(Reverse.FLYWHEEL_RPS, sp.flywheelRps(), EPS);
        assertEquals(Reverse.KICKER_DUTY, sp.kickerDuty(), EPS);
        assertEquals(Reverse.FLOOR_DUTY, sp.floorDuty(), EPS);
        assertEquals(Reverse.INTAKE_DUTY, sp.intakeDuty(), EPS);
        assertEquals(10.0, sp.turretDeg(), EPS, "turret keeps aiming");

        ss.setReverse(false);
        Setpoints after = at(0);
        assertEquals(52.0, after.flywheelRps(), EPS, "releasing reverse resumes whatever is still held");
        assertEquals(Intake.IN_DUTY, after.intakeDuty(), EPS);
    }

    @Test
    void intakeModes() {
        ss.setIntake(IntakeMode.IN);
        assertEquals(Intake.IN_DUTY, at(0).intakeDuty(), EPS);
        ss.setIntake(IntakeMode.OUT);
        assertEquals(Intake.OUT_DUTY, at(0).intakeDuty(), EPS);
    }

    @Test
    void rackFractions() {
        ss.setRack(1.0);
        assertEquals(Rack.EXTENDED_ROT, at(0).rackRot(), EPS);
        ss.setRack(0.5);
        assertEquals(Rack.EXTENDED_ROT / 2, at(0).rackRot(), EPS);
    }

    @Test
    void agitateCyclesWhileHeld() {
        ss.startAgitate(10.0);
        double[] expected = Rack.AGITATE_FRACTIONS;
        for (int i = 0; i < expected.length * 3; i++) {
            double t = 10.0 + i * Rack.AGITATE_STEP_S + 0.01;
            assertEquals(expected[i % expected.length] * Rack.EXTENDED_ROT, at(t).rackRot(), EPS, "step " + i);
        }
        assertTrue(ss.isAgitating());
    }

    @Test
    void releasingAgitateReturnsRackToWhereItWas() {
        ss.setRack(0.5);
        ss.startAgitate(0);
        at(0.9);
        ss.stopAgitate();
        assertFalse(ss.isAgitating());
        assertEquals(Rack.EXTENDED_ROT / 2, at(1.0).rackRot(), EPS);
    }

    @Test
    void agitateCanRunWhileShooting() {
        ss.setFeed(FeedMode.SHOOT);
        ss.startAgitate(0);
        Setpoints sp = at(0.01);
        assertEquals(52.0, sp.flywheelRps(), EPS);
        assertEquals(Rack.EXTENDED_ROT, sp.rackRot(), EPS);
    }

    @Test
    void rackButtonCancelsAgitate() {
        ss.startAgitate(0);
        at(0.1);
        ss.setRack(0);
        assertFalse(ss.isAgitating());
        assertEquals(Rack.RETRACTED_ROT, at(0.6).rackRot(), EPS);
    }

    @Test
    void hoodHomingZeroesAfterSustainedCurrentSpike() {
        ss.startHoodHoming(0);
        // Start-up spike is ignored.
        assertEquals(Hood.HOMING_VOLTS, hoodAmps(0.1, 50).hoodHomingVolts(), EPS);
        // Free-running, low current.
        assertEquals(Hood.HOMING_VOLTS, hoodAmps(0.5, 1).hoodHomingVolts(), EPS);
        // Hit the hard stop: must stay high for the debounce time.
        Setpoints first = hoodAmps(1.0, 20);
        assertFalse(first.zeroHood());
        Setpoints done = hoodAmps(1.0 + Hood.HOMING_DEBOUNCE_S + 0.01, 20);
        assertTrue(done.zeroHood());
        assertEquals(0, done.hoodHomingVolts());
        assertFalse(ss.isHoming());
        assertEquals("zeroed", ss.lastHomingResult());
    }

    @Test
    void hoodHomingGivesUpWithoutZeroing() {
        ss.startHoodHoming(0);
        Setpoints sp = hoodAmps(Hood.HOMING_TIMEOUT_S + 0.1, 0);
        assertFalse(sp.zeroHood());
        assertEquals(0, sp.hoodHomingVolts());
        assertFalse(ss.isHoming());
    }

    @Test
    void stopAllClearsLatchedModes() {
        ss.setFeed(FeedMode.SHOOT);
        ss.setIntake(IntakeMode.IN);
        ss.startAgitate(0);
        ss.setReverse(true);
        ss.stopAll();
        Setpoints sp = at(0);
        assertEquals(0, sp.flywheelRps());
        assertEquals(0, sp.intakeDuty());
        assertFalse(ss.isAgitating());
        assertFalse(ss.isReversing());
    }

    @Test
    void waitsForReadyBeforeFeedingButKeepsFlywheelSpinning() {
        ss.setFeed(FeedMode.SHOOT);
        Setpoints sp = ss.update(0, SHOT, NOT_READY);
        assertEquals(52.0, sp.flywheelRps(), EPS);
        assertEquals(0, sp.kickerDuty());
        assertEquals(0, ss.update(AFTER_UNJAM, SHOT, NOT_READY).floorDuty());
        assertEquals(Floor.FEED_DUTY, ss.update(AFTER_UNJAM + 0.1, SHOT, READY).floorDuty(), EPS);
    }

    @Test
    void onceFeedingItKeepsFeedingThroughTheBallDip() {
        ss.setFeed(FeedMode.SHOOT);
        ss.update(0, SHOT, READY);
        // A ball knocks the flywheel off speed: feeding must not stall.
        assertEquals(Floor.FEED_DUTY, ss.update(AFTER_UNJAM, SHOT, NOT_READY).floorDuty(), EPS);
    }

    @Test
    void feedsAnywayAfterTheTimeout() {
        ss.setFeed(FeedMode.SHOOT);
        ss.update(0, SHOT, NOT_READY);
        assertEquals(0, ss.update(Feed.GATE_TIMEOUT_S - 0.05, SHOT, NOT_READY).floorDuty());
        assertEquals(Floor.FEED_DUTY, ss.update(Feed.GATE_TIMEOUT_S + 0.01, SHOT, NOT_READY).floorDuty(), EPS);
    }

    @Test
    void waitForReadyCanBeTurnedOff() {
        ss.setWaitForReady(false);
        ss.setFeed(FeedMode.SHOOT);
        assertEquals(Kicker.FEED_DUTY, ss.update(0, SHOT, NOT_READY).kickerDuty(), EPS);
    }

    @Test
    void releasingShootResetsTheGate() {
        ss.setFeed(FeedMode.SHOOT);
        ss.update(0, SHOT, READY);
        ss.setFeed(FeedMode.OFF);
        at(0.5);
        ss.setFeed(FeedMode.SHOOT);
        assertEquals(0, ss.update(1.0, SHOT, NOT_READY).kickerDuty(), "new press waits for ready again");
    }

    @Test
    void holdFireStopsFeedingButNotTheFlywheel() {
        ss.setFeed(FeedMode.SHOOT);
        ss.setHoldFire(true);
        Setpoints sp = at(0);
        assertEquals(52.0, sp.flywheelRps(), EPS);
        assertEquals(0, sp.kickerDuty());
        assertEquals(0, at(AFTER_UNJAM).floorDuty());
        assertTrue(ss.isHoldingFire());
        ss.setHoldFire(false);
        assertEquals(Floor.FEED_DUTY, at(AFTER_UNJAM + 0.1).floorDuty(), EPS);
    }
}
