package frc.robot.shooting;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.Field;
import frc.robot.Constants.Hood;
import frc.robot.Constants.Shooting;
import frc.robot.Constants.Turret;
import frc.robot.Tuning;

/**
 * Pure function: robot state in, turret/hood/flywheel setpoints out. No hardware, no NetworkTables,
 * so every behavior here is covered by ShotCalculatorTest.
 *
 * <p>All math happens in the "own alliance" frame: blue-origin field coordinates, with the pose and
 * velocity flipped (180 degree rotation) when we are red. Targets are then defined once, for blue.
 */
public final class ShotCalculator {
    private ShotCalculator() {}

    /** What the driver sees on the dashboard. */
    public enum Mode { SCORING, PASSING, TRENCH }

    /**
     * @param target        what we are aiming at (hub or a pass target), own-alliance frame
     * @param distanceM     effective distance to the target after shoot-on-the-move lead
     * @param turretDeg     robot-relative turret command, wrapped into the turret's legal range
     * @param hoodUnits     hood command in legacy hood units (positive)
     * @param flywheelRps   flywheel velocity command
     * @param nearTrench    true when the hood was forced down to fit under a trench
     * @param tiltTurretDeg turret correction applied for chassis tilt (0 when level or disabled)
     * @param tiltHoodUnits hood correction applied for chassis tilt (0 when level or disabled)
     */
    public record Shot(
            Translation2d target,
            double distanceM,
            double turretDeg,
            double hoodUnits,
            double flywheelRps,
            boolean nearTrench,
            double tiltTurretDeg,
            double tiltHoodUnits) {

        public Mode mode() {
            if (nearTrench) return Mode.TRENCH;
            return target.equals(Field.HUB) ? Mode.SCORING : Mode.PASSING;
        }
    }

    /**
     * @param robotPose         blue-origin field pose (as the pose estimator reports it)
     * @param fieldVelocityMps  blue-origin field-relative robot velocity (vx, vy)
     * @param isRed             alliance
     * @param chassisTilt       robot roll/pitch/yaw from the gyro
     * @param tiltCompensation  correct turret + hood for a tilted chassis ("shoot on the bump")
     */
    public static Shot calculate(Pose2d robotPose, Translation2d fieldVelocityMps, boolean isRed,
            Rotation3d chassisTilt, boolean tiltCompensation) {
        Pose2d pose = isRed ? flip(robotPose) : robotPose;
        Translation2d velocity = isRed ? fieldVelocityMps.unaryMinus() : fieldVelocityMps;

        Translation2d turret = turretPosition(pose);
        Translation2d target = chooseTarget(turret);
        Translation2d toTarget = target.minus(turret);

        // Shoot on the move: the ball keeps the robot's velocity, so aim at where the target will be
        // relative to us after one time of flight.
        Translation2d aim = toTarget;
        for (int i = 0; i < Shooting.SOTM_ITERATIONS; i++) {
            aim = toTarget.minus(velocity.times(timeOfFlight(aim.getNorm())));
        }
        double distance = aim.getNorm();

        double fieldAzimuthRad = Math.atan2(aim.getY(), aim.getX());
        double robotYawRad = pose.getRotation().getRadians();
        double turretRad = fieldAzimuthRad - robotYawRad;

        boolean nearTrench = isNearTrench(turret.getX());
        double hoodUnits = nearTrench ? Hood.MIN_UNITS : hoodUnitsForDistance(distance);

        double tiltTurretDeg = 0, tiltHoodUnits = 0;
        if (tiltCompensation && !nearTrench
                && TiltCompensation.isTilted(chassisTilt, Math.toRadians(Shooting.TILT_MIN_DEG))) {
            double elevationRad = Math.toRadians(launchDegForHoodUnits(hoodUnits));
            tiltTurretDeg = MathUtil.clamp(
                    TiltCompensation.azimuthCorrectionDegrees(fieldAzimuthRad, elevationRad, chassisTilt, robotYawRad),
                    -Shooting.MAX_TILT_TURRET_CORRECTION_DEG, Shooting.MAX_TILT_TURRET_CORRECTION_DEG);
            double elevationFixDeg = TiltCompensation.elevationCorrectionDegrees(
                    turretRad + Math.toRadians(tiltTurretDeg), elevationRad, chassisTilt);
            double unitsPerLaunchDeg = Shooting.MORE_HOOD_UNITS_IS_FLATTER
                    ? -Tuning.HOOD_UNITS_PER_DEGREE.get() : Tuning.HOOD_UNITS_PER_DEGREE.get();
            tiltHoodUnits = MathUtil.clamp(elevationFixDeg * unitsPerLaunchDeg,
                    -Shooting.MAX_TILT_HOOD_CORRECTION_UNITS, Shooting.MAX_TILT_HOOD_CORRECTION_UNITS);
        }
        double hood = MathUtil.clamp(hoodUnits + tiltHoodUnits, Hood.MIN_UNITS, Hood.MAX_UNITS);

        return new Shot(target, distance, wrapTurret(Math.toDegrees(turretRad) + tiltTurretDeg), hood,
                flywheelRps(distance), nearTrench, tiltTurretDeg, hood - hoodUnits);
    }

    /** Estimated launch angle above horizontal for a hood command (see Constants.Shooting). */
    static double launchDegForHoodUnits(double hoodUnits) {
        double deltaDeg = (hoodUnits - Hood.MIN_UNITS) / Tuning.HOOD_UNITS_PER_DEGREE.get();
        return Tuning.LAUNCH_DEG_AT_MIN_HOOD.get() + (Shooting.MORE_HOOD_UNITS_IS_FLATTER ? -deltaDeg : deltaDeg);
    }

    /** Hub when we are in our alliance zone, otherwise the nearest-side pass target. */
    static Translation2d chooseTarget(Translation2d turret) {
        if (turret.getX() <= Field.TRENCH_X_M) {
            return Field.HUB;
        }
        return turret.getY() < Field.MID_Y_M ? Field.PASS_LOW : Field.PASS_HIGH;
    }

    /** The turret pivot is offset from robot center, so aim from there. */
    static Translation2d turretPosition(Pose2d pose) {
        double angle = pose.getRotation().getRadians() + Turret.PIVOT_ANGLE_RAD;
        return pose.getTranslation().plus(new Translation2d(
                Turret.PIVOT_RADIUS_M * Math.cos(angle), Turret.PIVOT_RADIUS_M * Math.sin(angle)));
    }

    static boolean isNearTrench(double x) {
        return Math.abs(x - Field.TRENCH_X_M) < Shooting.OUR_TRENCH_CLEARANCE_M
                || Math.abs(x - Field.FAR_TRENCH_X_M) < Shooting.FAR_TRENCH_CLEARANCE_M;
    }

    static double hoodUnitsForDistance(double distanceM) {
        return MathUtil.clamp(distanceM * Tuning.HOOD_UNITS_PER_METER.get(), Hood.MIN_UNITS, Hood.MAX_UNITS);
    }

    /** Wrap into [center - 180, center + 180) so the turret never has to cross its hard limit. */
    static double wrapTurret(double deg) {
        return MathUtil.inputModulus(deg, Turret.WRAP_CENTER_DEG - 180, Turret.WRAP_CENTER_DEG + 180);
    }

    public static double flywheelRps(double distanceM) {
        return interpolate(distanceM, 1);
    }

    public static double timeOfFlight(double distanceM) {
        return interpolate(distanceM, 2);
    }

    /** Linear interpolation on the shot table (live-tunable values), clamped at both ends. */
    private static double interpolate(double distanceM, int col) {
        double[][] t = Shooting.SHOT_TABLE;
        Tuning.Value[] y = col == 1 ? Tuning.SHOT_RPS : Tuning.SHOT_TOF_S;
        if (distanceM <= t[0][0]) return y[0].get();
        for (int i = 1; i < t.length; i++) {
            if (distanceM <= t[i][0]) {
                double f = (distanceM - t[i - 1][0]) / (t[i][0] - t[i - 1][0]);
                return y[i - 1].get() + f * (y[i].get() - y[i - 1].get());
            }
        }
        return y[t.length - 1].get();
    }

    /** Rotate a blue-origin pose into red's point of view (the 2026 field is rotationally symmetric). */
    public static Pose2d flip(Pose2d pose) {
        return new Pose2d(Field.LENGTH_M - pose.getX(), Field.WIDTH_M - pose.getY(),
                pose.getRotation().rotateBy(Rotation2d.kPi));
    }
}
