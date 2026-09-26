package frc.robot.shooting;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation3d;

import org.junit.jupiter.api.Test;

/**
 * Unit tests for {@link TiltCompensation} ("shoot on the bump" math).
 *
 * <p>
 * These are pure-math tests: no HAL, no hardware. The important property is that applying the
 * computed correction makes the projectile's field-frame direction actually point where we want,
 * and that everything degenerates to zero when the chassis is level.
 */
class TiltCompensationTest {

    private static final double EPS = 1e-9;

    @Test
    void levelChassisProducesNoCorrections() {
        Rotation3d level = new Rotation3d();
        double desiredWorldAz = Math.toRadians(30);
        double elev = Math.toRadians(20);
        double yaw = Math.toRadians(15);

        // On flat ground the correction must be exactly zero so enabling this is a no-op.
        assertEquals(0.0, TiltCompensation.azimuthCorrectionDegrees(desiredWorldAz, elev, level, yaw), EPS,
                "Azimuth correction should be 0 when level");
        assertEquals(0.0, TiltCompensation.elevationCorrectionDegrees(yaw, elev, level), EPS,
                "Elevation correction should be 0 when level");
        assertFalse(TiltCompensation.isTilted(level, TiltCompensation.DEFAULT_BUMP_THRESHOLD_RAD));
    }

    @Test
    void azimuthSolveHitsDesiredGroundTrackWhenPitched() {
        // Robot pitched on the bump, arbitrary yaw.
        Rotation3d tilt = new Rotation3d(0, Math.toRadians(12), Math.toRadians(-25));
        double desiredWorldAz = Math.toRadians(135);
        double elev = Math.toRadians(22);
        double yaw = Math.toRadians(20);

        double correctedAz = TiltCompensation.turretAzimuthForWorldAzimuth(desiredWorldAz, elev, tilt, yaw);
        double achievedWorldAz = TiltCompensation.worldAzimuth(correctedAz, elev, tilt, yaw);

        // After the solve, the projectile's ground track should match what we asked for.
        double err = Math.abs(MathUtil.angleModulus(achievedWorldAz - desiredWorldAz));
        assertTrue(err < Math.toRadians(0.05),
                "Solved azimuth should hit desired ground track, error (deg) = " + Math.toDegrees(err));
    }

    @Test
    void azimuthCorrectionImprovesPointingWhenPitched() {
        Rotation3d tilt = new Rotation3d(0, Math.toRadians(12), 0.0);
        double desiredWorldAz = Math.toRadians(90);
        double elev = Math.toRadians(20);
        double yaw = 0.0;

        double correction = TiltCompensation.azimuthCorrectionDegrees(desiredWorldAz, elev, tilt, yaw);
        assertTrue(Math.abs(correction) > 0.1,
                "A pitched chassis should require a non-trivial azimuth correction, got " + correction + " deg");

        // Applying the correction should reduce the pointing error versus not applying it.
        double flat = MathUtil.angleModulus(desiredWorldAz - yaw);
        double errNoComp = Math.abs(MathUtil.angleModulus(
                TiltCompensation.worldAzimuth(flat, elev, tilt, yaw) - desiredWorldAz));
        double errComp = Math.abs(MathUtil.angleModulus(
                TiltCompensation.worldAzimuth(flat + Math.toRadians(correction), elev, tilt, yaw) - desiredWorldAz));
        assertTrue(errComp < errNoComp, "Correction should improve pointing accuracy");
        assertTrue(errComp < Math.toRadians(0.05), "Corrected error should be small");
    }

    @Test
    void elevationCorrectionRestoresVerticalLaunchAngle() {
        Rotation3d tilt = new Rotation3d(0, Math.toRadians(10), 0.0);
        double turretAz = Math.toRadians(45);
        double commandedElev = Math.toRadians(25);

        double correctedElev = commandedElev + Math.toRadians(
                TiltCompensation.elevationCorrectionDegrees(turretAz, commandedElev, tilt));
        double achieved = TiltCompensation.worldElevation(turretAz, correctedElev, tilt);

        assertEquals(Math.toDegrees(commandedElev), Math.toDegrees(achieved), 0.5,
                "Corrected elevation should restore the commanded vertical launch angle");
    }

    @Test
    void bumpDetectionThreshold() {
        Rotation3d smallTilt = new Rotation3d(0, Math.toRadians(1), 0);
        Rotation3d bigTilt = new Rotation3d(0, Math.toRadians(8), 0);
        assertFalse(TiltCompensation.isTilted(smallTilt, TiltCompensation.DEFAULT_BUMP_THRESHOLD_RAD));
        assertTrue(TiltCompensation.isTilted(bigTilt, TiltCompensation.DEFAULT_BUMP_THRESHOLD_RAD));
    }
}