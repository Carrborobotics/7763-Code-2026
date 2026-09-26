package frc.robot.shooting;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.Field;
import frc.robot.Constants.Hood;
import frc.robot.Constants.Shooting;
import frc.robot.shooting.ShotCalculator.Shot;

class ShotCalculatorTest {
    private static final double EPS = 1e-6;
    private static final Translation2d STILL = Translation2d.kZero;
    private static final Rotation3d LEVEL = new Rotation3d();

    private static Shot blue(Pose2d pose, Translation2d velocity) {
        return ShotCalculator.calculate(pose, velocity, false, LEVEL, false);
    }

    /** Field-frame direction the turret is pointing, in degrees. */
    private static double fieldAimDeg(Shot shot, Pose2d pose) {
        return MathUtil.inputModulus(shot.turretDeg() + pose.getRotation().getDegrees(), -180, 180);
    }

    @Test
    void stationaryShotAimsTurretPivotAtHub() {
        Pose2d pose = new Pose2d(2.0, Field.MID_Y_M, Rotation2d.kZero);
        Shot shot = blue(pose, STILL);

        Translation2d pivot = ShotCalculator.turretPosition(pose);
        Translation2d toHub = Field.HUB.minus(pivot);
        assertEquals(Field.HUB, shot.target());
        assertEquals(toHub.getNorm(), shot.distanceM(), EPS);
        assertEquals(toHub.getAngle().getDegrees(), shot.turretDeg(), EPS);
        assertEquals(ShotCalculator.flywheelRps(toHub.getNorm()), shot.flywheelRps(), EPS);
        assertEquals(toHub.getNorm() * Hood.UNITS_PER_METER, shot.hoodUnits(), EPS);
    }

    @Test
    void turretCommandCompensatesForRobotHeading() {
        Pose2d facingForward = new Pose2d(2.0, 2.0, Rotation2d.kZero);
        Pose2d facingLeft = new Pose2d(2.0, 2.0, Rotation2d.kCCW_Pi_2);
        // Pivot moves when the robot rotates, so compare field-frame aim to the pivot->hub angle.
        for (Pose2d pose : new Pose2d[] {facingForward, facingLeft}) {
            Shot shot = blue(pose, STILL);
            double expected = Field.HUB.minus(ShotCalculator.turretPosition(pose)).getAngle().getDegrees();
            assertEquals(expected, fieldAimDeg(shot, pose), EPS);
        }
    }

    @Test
    void redAllianceIsTheRotatedMirrorOfBlue() {
        Pose2d bluePose = new Pose2d(2.5, 1.5, Rotation2d.fromDegrees(30));
        Translation2d blueVel = new Translation2d(0.4, -0.3);
        Shot b = blue(bluePose, blueVel);

        Pose2d redPose = ShotCalculator.flip(bluePose);
        Translation2d redVel = blueVel.unaryMinus();
        Shot r = ShotCalculator.calculate(redPose, redVel, true, LEVEL, false);

        assertEquals(b.target(), r.target());
        assertEquals(b.distanceM(), r.distanceM(), EPS);
        assertEquals(b.turretDeg(), r.turretDeg(), EPS);
        assertEquals(b.hoodUnits(), r.hoodUnits(), EPS);
        assertEquals(b.flywheelRps(), r.flywheelRps(), EPS);
    }

    @Test
    void pastOurTrenchWePassToTheNearSide() {
        Shot low = blue(new Pose2d(8.0, 1.0, Rotation2d.kZero), STILL);
        Shot high = blue(new Pose2d(8.0, Field.WIDTH_M - 1.0, Rotation2d.kZero), STILL);
        assertEquals(Field.PASS_LOW, low.target());
        assertEquals(Field.PASS_HIGH, high.target());
    }

    @Test
    void hoodDropsNearEitherTrench() {
        assertTrue(ShotCalculator.isNearTrench(Field.TRENCH_X_M));
        assertTrue(ShotCalculator.isNearTrench(Field.FAR_TRENCH_X_M + 0.5));
        assertFalse(ShotCalculator.isNearTrench(Field.TRENCH_X_M - 1.0));
        assertFalse(ShotCalculator.isNearTrench(Field.LENGTH_M / 2));

        // Put the turret pivot right on our trench line.
        Pose2d pose = new Pose2d(Field.TRENCH_X_M - 0.187, 1.0, Rotation2d.kZero);
        Shot shot = blue(pose, STILL);
        assertTrue(shot.nearTrench());
        assertEquals(Hood.MIN_UNITS, shot.hoodUnits(), EPS);
    }

    @Test
    void hoodIsClampedToItsRange() {
        assertEquals(Hood.MIN_UNITS, ShotCalculator.hoodUnitsForDistance(0.5), EPS);
        assertEquals(Hood.MAX_UNITS, ShotCalculator.hoodUnitsForDistance(20), EPS);
        assertEquals(3.0 * Hood.UNITS_PER_METER, ShotCalculator.hoodUnitsForDistance(3.0), EPS);
    }

    @Test
    void shotTableInterpolatesAndClamps() {
        assertEquals(40.75, ShotCalculator.flywheelRps(1.25), EPS);
        assertEquals(39.0, ShotCalculator.flywheelRps(0.0), EPS);
        assertEquals(80.0, ShotCalculator.flywheelRps(99.0), EPS);
        assertEquals(0.725, ShotCalculator.timeOfFlight(2.5), EPS);
    }

    @Test
    void turretCommandStaysInsideItsLegalRange() {
        for (int deg = -720; deg <= 720; deg += 5) {
            double wrapped = ShotCalculator.wrapTurret(deg);
            assertTrue(wrapped >= -95 - EPS && wrapped < 265 + EPS, "wrapped " + deg + " to " + wrapped);
            assertEquals(0, MathUtil.inputModulus(wrapped - deg, -180, 180), EPS);
        }
    }

    @Test
    void movingTowardTheHubShortensTheShot() {
        Pose2d pose = new Pose2d(1.5, Field.MID_Y_M, Rotation2d.kZero);
        Shot still = blue(pose, STILL);
        Shot moving = blue(pose, new Translation2d(1.0, 0));
        assertTrue(moving.distanceM() < still.distanceM());
        assertEquals(moving.flywheelRps(), ShotCalculator.flywheelRps(moving.distanceM()), EPS,
                "flywheel must use the same lead-compensated distance as the hood");
    }

    @Test
    void strafingLeadsOppositeToOurVelocity() {
        Pose2d pose = new Pose2d(1.5, Field.MID_Y_M, Rotation2d.kZero);
        double stillAim = fieldAimDeg(blue(pose, STILL), pose);
        double movingAim = fieldAimDeg(blue(pose, new Translation2d(0, 1.0)), pose);
        assertTrue(movingAim < stillAim, "moving +y should aim further toward -y");
    }

    /** The old code used robot-relative velocity here, which was only right when facing downfield. */
    @Test
    void leadDependsOnFieldVelocityNotRobotHeading() {
        Translation2d velocity = new Translation2d(0.3, 0.8);
        Pose2d a = new Pose2d(1.5, 2.0, Rotation2d.kZero);
        Pose2d b = new Pose2d(1.5, 2.0, Rotation2d.fromDegrees(90));
        Shot sa = blue(a, velocity);
        Shot sb = blue(b, velocity);
        // Lead offset relative to a stationary shot must be the same regardless of heading.
        double leadA = fieldAimDeg(sa, a) - fieldAimDeg(blue(a, STILL), a);
        double leadB = fieldAimDeg(sb, b) - fieldAimDeg(blue(b, STILL), b);
        assertEquals(leadA, leadB, 0.5); // pivot offset moves slightly with heading
    }

    @Test
    void modeSaysScoringPassingOrTrench() {
        assertEquals(ShotCalculator.Mode.SCORING, blue(new Pose2d(2.0, 2.0, Rotation2d.kZero), STILL).mode());
        assertEquals(ShotCalculator.Mode.PASSING, blue(new Pose2d(8.0, 2.0, Rotation2d.kZero), STILL).mode());
        Pose2d underTrench = new Pose2d(Field.TRENCH_X_M - 0.187, 1.0, Rotation2d.kZero);
        assertEquals(ShotCalculator.Mode.TRENCH, blue(underTrench, STILL).mode());
    }

    private static Shot tilted(Pose2d pose, Rotation3d tilt, boolean enabled) {
        return ShotCalculator.calculate(pose, STILL, false, tilt, enabled);
    }

    @Test
    void tiltCompensationDoesNothingOnFlatGroundOrSmallNoise() {
        Pose2d pose = new Pose2d(2.0, 2.0, Rotation2d.fromDegrees(20));
        Rotation3d noise = new Rotation3d(Math.toRadians(1), Math.toRadians(-1.5), 0);
        Shot base = tilted(pose, LEVEL, false);
        for (Rotation3d tilt : new Rotation3d[] {LEVEL, noise}) {
            Shot s = tilted(pose, tilt, true);
            assertEquals(base.turretDeg(), s.turretDeg(), EPS);
            assertEquals(base.hoodUnits(), s.hoodUnits(), EPS);
            assertEquals(0, s.tiltTurretDeg(), EPS);
        }
    }

    @Test
    void tiltCompensationCorrectsOnTheBumpWithinItsClamps() {
        Pose2d pose = new Pose2d(2.0, 2.0, Rotation2d.fromDegrees(20));
        Rotation3d bump = new Rotation3d(Math.toRadians(10), Math.toRadians(8), 0);
        Shot off = tilted(pose, bump, false);
        Shot on = tilted(pose, bump, true);
        assertTrue(Math.abs(on.tiltTurretDeg()) > 0.1 || Math.abs(on.tiltHoodUnits()) > 0.1, "should correct");
        assertTrue(Math.abs(on.tiltTurretDeg()) <= Shooting.MAX_TILT_TURRET_CORRECTION_DEG + EPS);
        assertTrue(Math.abs(on.tiltHoodUnits()) <= Shooting.MAX_TILT_HOOD_CORRECTION_UNITS + EPS);
        assertEquals(off.flywheelRps(), on.flywheelRps(), EPS, "tilt never changes flywheel speed");
        assertTrue(on.hoodUnits() >= Hood.MIN_UNITS && on.hoodUnits() <= Hood.MAX_UNITS);
    }

    @Test
    void tiltCompensationNeverLiftsTheHoodUnderTheTrench() {
        Pose2d underTrench = new Pose2d(Field.TRENCH_X_M - 0.187, 1.0, Rotation2d.kZero);
        Shot s = tilted(underTrench, new Rotation3d(0, Math.toRadians(12), 0), true);
        assertEquals(Hood.MIN_UNITS, s.hoodUnits(), EPS);
    }

    @Test
    void hoodGeometryEstimateIsSane() {
        double steep = ShotCalculator.launchDegForHoodUnits(Hood.MIN_UNITS);
        double flat = ShotCalculator.launchDegForHoodUnits(Hood.MAX_UNITS);
        assertEquals(Shooting.LAUNCH_DEG_AT_MIN_HOOD, steep, EPS);
        assertTrue(flat > 20 && flat < 85, "launch angle at max hood " + flat);
    }
}
