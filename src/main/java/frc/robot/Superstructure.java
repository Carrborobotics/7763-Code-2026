package frc.robot;

import frc.robot.Constants.Floor;
import frc.robot.Constants.Hood;
import frc.robot.Constants.Intake;
import frc.robot.Constants.Kicker;
import frc.robot.Constants.Rack;
import frc.robot.shooting.ShotCalculator.Shot;

/**
 * The one place that decides what every mechanism (everything except the drivetrain) should do.
 *
 * <p>Driver buttons and auto named commands only change the requested modes below. Once per loop
 * {@link #update} turns those modes plus the current shot solution into a {@link Setpoints} record,
 * and Robot hands that record to the hardware. This class never touches hardware, so its behavior is
 * covered by SuperstructureTest.
 */
public final class Superstructure {

    public enum IntakeMode { OFF, IN, OUT }

    /** OFF: flywheel coasts. SHOOT: spin up and feed. */
    public enum FeedMode { OFF, SHOOT }

    /** Sensor readings the state machine needs each loop. */
    public record Measured(double hoodStatorAmps, boolean flywheelAtSpeed, double turretErrorDeg) {
        public static final Measured NONE = new Measured(0, false, 0);
    }

    /**
     * Everything the hardware needs this loop.
     *
     * @param hoodHomingVolts if non-zero, run the hood open-loop at this voltage instead of hoodUnits
     * @param zeroHood        homing just finished: reset the hood encoder to 0 this loop
     * @param flywheelRps     0 means coast
     */
    public record Setpoints(
            double turretDeg,
            double hoodUnits,
            double hoodHomingVolts,
            boolean zeroHood,
            double flywheelRps,
            double kickerDuty,
            double floorDuty,
            double intakeDuty,
            double rackRot) {}

    private IntakeMode intakeMode = IntakeMode.OFF;
    private FeedMode feedMode = FeedMode.OFF;
    private double rackGoalRot = Rack.RETRACTED_ROT;

    private boolean reversing = false;
    private boolean waitForReady = Constants.Feed.WAIT_FOR_READY;
    private boolean holdFire = false;
    private double shootStartS = Double.NaN;
    private boolean feeding = false;
    private boolean ready = false;

    private double agitateStartS = Double.NaN;
    private double rackGoalBeforeAgitateRot = Rack.RETRACTED_ROT;
    private double homingStartS = Double.NaN;
    private double homingCurrentSinceS = Double.NaN;
    private String lastHomingResult = "not run";

    private double flywheelOffsetRps = 0;
    private double turretOffsetDeg = 0;

    // ---- requests (from Robot's button bindings and auto named commands) ----

    public void setIntake(IntakeMode mode) {
        intakeMode = mode;
    }

    public void setFeed(FeedMode mode) {
        feedMode = mode;
    }

    /** Rack to a fraction of full extension (0 = retracted, 1 = extended). Cancels agitate. */
    public void setRack(double fractionExtended) {
        agitateStartS = Double.NaN;
        rackGoalRot = rackRotFor(fractionExtended);
    }

    private static double rackRotFor(double fractionExtended) {
        return Rack.RETRACTED_ROT + fractionExtended * (Rack.EXTENDED_ROT - Rack.RETRACTED_ROT);
    }

    /** Hold the floor + kicker until the flywheel is at speed and the turret is on target (with a timeout). */
    public void setWaitForReady(boolean wait) {
        waitForReady = wait;
    }

    /** Don't feed while true (e.g. our hub is inactive). The flywheel keeps spinning so we fire instantly after. */
    public void setHoldFire(boolean hold) {
        holdFire = hold;
    }

    /** Every roller runs backwards while true. Overrides shoot and intake; turret/hood/rack are unaffected. */
    public void setReverse(boolean reverse) {
        reversing = reverse;
    }

    /** Start shaking the rack. Runs until {@link #stopAgitate}, then the rack returns to where it was. */
    public void startAgitate(double nowS) {
        if (!isAgitating()) rackGoalBeforeAgitateRot = rackGoalRot;
        agitateStartS = nowS;
    }

    public void stopAgitate() {
        if (isAgitating()) rackGoalRot = rackGoalBeforeAgitateRot;
        agitateStartS = Double.NaN;
    }

    public void startHoodHoming(double nowS) {
        homingStartS = nowS;
        homingCurrentSinceS = Double.NaN;
    }

    public void adjustFlywheelOffset(double deltaRps) {
        flywheelOffsetRps += deltaRps;
    }

    public void adjustTurretOffset(double deltaDeg) {
        turretOffsetDeg += deltaDeg;
    }

    /** Stop every roller and cancel timed routines. Used on disable and at the start of teleop. */
    public void stopAll() {
        intakeMode = IntakeMode.OFF;
        feedMode = FeedMode.OFF;
        reversing = false;
        stopAgitate();
        homingStartS = Double.NaN;
    }

    // ---- the state machine ----

    public Setpoints update(double nowS, Shot shot, Measured measured) {
        double rackRot = updateRack(nowS);
        double hoodStatorCurrentA = measured.hoodStatorAmps();

        double homingVolts = 0;
        boolean zeroHood = false;
        if (isHoming()) {
            double elapsed = nowS - homingStartS;
            boolean stalled = elapsed > Hood.HOMING_IGNORE_S && hoodStatorCurrentA > Hood.HOMING_CURRENT_A;
            if (!stalled) {
                homingCurrentSinceS = Double.NaN;
            } else if (Double.isNaN(homingCurrentSinceS)) {
                homingCurrentSinceS = nowS;
            }

            if (stalled && nowS - homingCurrentSinceS >= Hood.HOMING_DEBOUNCE_S) {
                zeroHood = true;
                homingStartS = Double.NaN;
                lastHomingResult = "zeroed";
            } else if (elapsed > Hood.HOMING_TIMEOUT_S) {
                homingStartS = Double.NaN; // give up without zeroing; the old zero is still better than a guess
                lastHomingResult = "timed out (no current spike)";
            } else {
                homingVolts = Hood.HOMING_VOLTS;
            }
        }

        double flywheel = 0, kicker = 0, floor = 0, intake = 0;
        ready = measured.flywheelAtSpeed()
                && Math.abs(measured.turretErrorDeg()) <= Tuning.TURRET_ON_TARGET_DEG.get();
        if (feedMode == FeedMode.SHOOT) {
            flywheel = Math.max(0, shot.flywheelRps() + flywheelOffsetRps);
            if (Double.isNaN(shootStartS)) shootStartS = nowS;
            // Once feeding starts it stays on: a ball going through dips the flywheel, which must not stall the feed.
            if (!feeding) {
                feeding = !waitForReady || ready || nowS - shootStartS >= Tuning.FEED_GATE_TIMEOUT_S.get();
            }
            if (feeding && !holdFire) {
                kicker = Kicker.FEED_DUTY;
                floor = Floor.FEED_DUTY;
            }
        } else {
            shootStartS = Double.NaN;
            feeding = false;
        }
        switch (intakeMode) {
            case IN -> intake = Intake.IN_DUTY;
            case OUT -> intake = Intake.OUT_DUTY;
            case OFF -> {}
        }
        if (reversing) {
            flywheel = Tuning.REVERSE_FLYWHEEL_RPS.get();
            kicker = Tuning.REVERSE_KICKER.get();
            floor = Tuning.REVERSE_FLOOR.get();
            intake = Tuning.REVERSE_INTAKE.get();
        }

        return new Setpoints(shot.turretDeg() + turretOffsetDeg, shot.hoodUnits(), homingVolts, zeroHood,
                flywheel, kicker, floor, intake, rackRot);
    }

    private double updateRack(double nowS) {
        if (isAgitating()) {
            double[] fractions = Rack.AGITATE_FRACTIONS;
            int step = (int) ((nowS - agitateStartS) / Rack.AGITATE_STEP_S) % fractions.length;
            return rackRotFor(fractions[step]);
        }
        return rackGoalRot;
    }

    // ---- telemetry getters ----

    public IntakeMode intakeMode() {
        return intakeMode;
    }

    public FeedMode feedMode() {
        return feedMode;
    }

    public boolean isHoming() {
        return !Double.isNaN(homingStartS);
    }

    /** Flywheel at speed and turret on target this loop. */
    public boolean isReady() {
        return ready;
    }

    /** Actually pushing balls into the shooter this loop. */
    public boolean isFeeding() {
        return feedMode == FeedMode.SHOOT && feeding && !holdFire && !reversing;
    }

    public boolean isHoldingFire() {
        return feedMode == FeedMode.SHOOT && holdFire;
    }

    public boolean isReversing() {
        return reversing;
    }

    public boolean isAgitating() {
        return !Double.isNaN(agitateStartS);
    }

    public String lastHomingResult() {
        return lastHomingResult;
    }

    public double flywheelOffsetRps() {
        return flywheelOffsetRps;
    }

    public double turretOffsetDeg() {
        return turretOffsetDeg;
    }
}
