package frc.robot.shooting;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * Pure-math helpers for "shoot on the bump": compensating a turret + hood shot for the
 * chassis being tilted (roll/pitch) while on the bump.
 *
 * <p>
 * The problem: the turret + hood point a fixed direction <b>in the robot frame</b>, but the
 * projectile actually flies along that direction <b>in the field frame</b>. When the robot is
 * level these are the same, so a naive {@code fieldAzimuth - robotYaw} turret command works.
 * On the bump they are not: the tilted chassis rotates the launch vector, which both skews the
 * projectile's ground-track azimuth and changes its vertical launch angle. This class computes
 * the corrections needed so the projectile's <i>field-frame</i> launch direction still points at
 * the target.
 *
 * <p>
 * All methods here are stateless and free of hardware, so they can be unit tested directly.
 * Every correction returns exactly {@code 0} when the chassis is level, which means enabling this
 * on flat ground is a no-op.
 */
public final class TiltCompensation {

    private TiltCompensation() {}

    /** Tilt (roll/pitch magnitude) above which the robot is considered "on the bump", in radians. */
    public static final double DEFAULT_BUMP_THRESHOLD_RAD = Math.toRadians(3.0);

    /**
     * Field-frame unit launch vector for a shot fired at {@code turretAzimuthRad} and
     * {@code elevationRad} from a chassis oriented as {@code chassisTilt}.
     *
     * @param turretAzimuthRad turret azimuth in the robot frame (0 = robot +X / forward), radians
     * @param elevationRad     hood elevation above the robot's XY plane, radians
     * @param chassisTilt      full robot orientation in the field frame (roll, pitch, yaw)
     * @return non-normalized (unit-length) launch direction in the field frame
     */
    public static Translation3d launchVectorField(double turretAzimuthRad, double elevationRad,
            Rotation3d chassisTilt) {
        double ca = Math.cos(turretAzimuthRad);
        double sa = Math.sin(turretAzimuthRad);
        double cb = Math.cos(elevationRad);
        double sb = Math.sin(elevationRad);
        // Robot-frame launch direction. Only roll/pitch tilt the launch vector; yaw is supplied
        // separately (see worldAzimuth) so it is never double-counted.
        Translation3d robotFrame = new Translation3d(cb * ca, cb * sa, sb);
        return robotFrame.rotateBy(new Rotation3d(chassisTilt.getX(), chassisTilt.getY(), 0.0));
    }

    /**
     * Azimuth of the launch vector in the yaw-free tilted robot frame. Add the robot yaw to get the
     * field-frame ground track.
     */
    public static double relativeAzimuth(double turretAzimuthRad, double elevationRad, Rotation3d chassisTilt) {
        Translation3d v = launchVectorField(turretAzimuthRad, elevationRad, chassisTilt);
        return Math.atan2(v.getY(), v.getX());
    }

    /**
     * Azimuth of the projectile's ground track in the field frame, radians. The robot's yaw is
     * applied here (exactly once) so the turret solve can reference the field.
     *
     * @see #launchVectorField(double, double, Rotation3d)
     */
    public static double worldAzimuth(double turretAzimuthRad, double elevationRad, Rotation3d chassisTilt,
            double robotYawRad) {
        return MathUtil.angleModulus(robotYawRad + relativeAzimuth(turretAzimuthRad, elevationRad, chassisTilt));
    }

    /**
     * Vertical launch angle of the projectile in the field frame, radians.
     *
     * @see #launchVectorField(double, double, Rotation3d)
     */
    public static double worldElevation(double turretAzimuthRad, double elevationRad, Rotation3d chassisTilt) {
        Translation3d v = launchVectorField(turretAzimuthRad, elevationRad, chassisTilt);
        double norm = v.getNorm();
        if (norm < 1e-9) {
            return 0.0;
        }
        return Math.asin(MathUtil.clamp(v.getZ() / norm, -1.0, 1.0));
    }

    /**
     * Solve for the turret azimuth command (robot frame, radians) that makes the projectile's
     * ground track lie along {@code desiredWorldAzimuthRad}.
     *
     * <p>
     * Uses a flat-ground initial guess followed by a few fixed-point refinements. This converges
     * very quickly for realistic bump tilts (< ~20&deg;) and degenerates to
     * {@code desiredWorldAzimuthRad - robotYawRad} when level.
     *
     * @param desiredWorldAzimuthRad field-frame azimuth we want the projectile to travel along
     * @param elevationRad           hood elevation used for the solve, radians
     * @param chassisTilt            full robot orientation in the field frame
     * @param robotYawRad            robot heading, radians
     * @return turret azimuth command in the robot frame, radians
     */
    public static double turretAzimuthForWorldAzimuth(double desiredWorldAzimuthRad, double elevationRad,
            Rotation3d chassisTilt, double robotYawRad) {
        // Flat-ground guess: this is exactly what the existing (level) code computes.
        double azimuth = MathUtil.angleModulus(desiredWorldAzimuthRad - robotYawRad);
        for (int i = 0; i < 5; i++) {
            double err = MathUtil.angleModulus(
                    desiredWorldAzimuthRad - worldAzimuth(azimuth, elevationRad, chassisTilt, robotYawRad));
            azimuth = MathUtil.angleModulus(azimuth + err);
        }
        return azimuth;
    }

    /**
     * Additive turret azimuth correction, in degrees, to add to the existing flat-ground turret
     * command so the projectile still travels at {@code desiredWorldAzimuthRad}. Returns 0 when
     * level.
     */
    public static double azimuthCorrectionDegrees(double desiredWorldAzimuthRad, double elevationRad,
            Rotation3d chassisTilt, double robotYawRad) {
        double corrected = turretAzimuthForWorldAzimuth(desiredWorldAzimuthRad, elevationRad, chassisTilt,
                robotYawRad);
        double flat = MathUtil.angleModulus(desiredWorldAzimuthRad - robotYawRad);
        return Math.toDegrees(MathUtil.angleModulus(corrected - flat));
    }

    /**
     * Additive world-elevation correction, in degrees, needed so the projectile's vertical launch
     * angle is preserved at {@code commandedElevationRad} despite the chassis tilt. Returns 0 when
     * level.
     *
     * <p>
     * This is a first-order correction (the derivative of the achieved elevation w.r.t. the hood
     * command is ~1 for realistic tilts), which is accurate to well under a degree for bump-sized
     * angles.
     */
    public static double elevationCorrectionDegrees(double turretAzimuthRad, double commandedElevationRad,
            Rotation3d chassisTilt) {
        double achieved = worldElevation(turretAzimuthRad, commandedElevationRad, chassisTilt);
        return Math.toDegrees(commandedElevationRad - achieved);
    }

    /**
     * @return true if the roll/pitch magnitude of {@code chassisTilt} exceeds {@code thresholdRad}.
     */
    public static boolean isTilted(Rotation3d chassisTilt, double thresholdRad) {
        return Math.hypot(chassisTilt.getX(), chassisTilt.getY()) > thresholdRad;
    }
}