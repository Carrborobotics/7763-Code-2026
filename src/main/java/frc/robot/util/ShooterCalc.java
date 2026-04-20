package frc.robot.util;

import java.util.List;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants;
import frc.robot.subsystems.swerve.Swerve;

public class ShooterCalc {

    /** Targeting algorithm mode selector. */
    public enum TargetingMode {
        /** Traditional static targeting (no motion compensation) */ BASIC, 
        /** Shoot-on-the-move w/ single-pass distance calc (fast) */ SOTM, 
        /** Shoot-on-the-move w/ iterative convergence (accurate) */ SOTM_CONVERGED
    }

    private Swerve s_Swerve;
    private TargetingMode targetingMode;

    public ShooterCalc(Swerve s_Swerve, TargetingMode targetingMode) {
        this.s_Swerve = s_Swerve;
        this.targetingMode = targetingMode;
    }

    /**
     * Set the targeting algorithm mode at runtime.
     */
    public void setTargetingMode(TargetingMode mode) {
        this.targetingMode = mode;
    }

    // Shoot on the move scale factor
    public double SOTM_SCALAR = -1.5 ; // multiple of robot velocity vector; should be -1.0 to subtract 

    /**
     * Get the current targeting algorithm mode.
     */
    public TargetingMode getTargetingMode() {
        return this.targetingMode;
    }

    public Pose2d getRelativeRobotPose() {
        return Swerve.flipIfRed(s_Swerve.getRobotPose());
    }

    /* 
    * Chooses the hub target or the pass-zones.
    * Assumes the robotPose is already flipped for red/blue sides
    */
    public Translation2d getTargetForRobotPose(Pose2d robotPose) {
        Translation2d target = Constants.Localization.hubPosition;
        if (robotPose.getX() > Constants.Localization.trenchline) {
            if (robotPose.getY() < Constants.Localization.yMidline) {
                target = Constants.Localization.lowerPassTarget;
            } else {
                target = Constants.Localization.upperPassTarget;
            }
        }
        return target;
    }

    // SmartDashboard.putString("Target/rpose-for-dist", robotPose.toString());
    public Translation2d getVectorToTarget() {
        Pose2d robotPose = getRelativeRobotPose();
        Translation2d target = getTargetForRobotPose(robotPose);
        return target.minus(robotPose.getTranslation());
    }
    
    //SmartDashboard.putNumber("Target/dist", targetDistance);   
    public double getTargetDistance() {
        Translation2d toTarget = this.getVectorToTarget();
        return Math.abs(Math.hypot(toTarget.getX(), toTarget.getY()));
    }

    public Translation2d getVectorToTargetSOTM0() {
        Translation2d realTargetGoal = this.getVectorToTarget();
        // get robot's current x and y velocity
        ChassisSpeeds speeds = s_Swerve.getSpeeds();
        double robotVx = speeds.vxMetersPerSecond;
        double robotVy = speeds.vyMetersPerSecond;
        double timeOfFlight = getTimeOfFlight(getTargetDistance());
        Translation2d robotMovementVector = new Translation2d(robotVx * timeOfFlight, robotVy * timeOfFlight);
        // for shoot on the the move we take the realTargetGoal and add the opposite of the robot
        // movement vector to it so that we are effectively calculating the distance to where the
        // target will be when the robot reaches it, not where it is now.
        Translation2d adjustedTargetGoal = realTargetGoal.plus(robotMovementVector.times(SOTM_SCALAR));
        return adjustedTargetGoal;
    }

    public double getTargetDistanceSOTM0() {
        Translation2d targetGoal = this.getVectorToTarget();
        double targetDistance = Math.abs(Math.hypot(targetGoal.getX(), targetGoal.getY()));
        SmartDashboard.putNumber("Target/dist-non-sotm", targetDistance);   
        
        Translation2d adjustedTargetGoal = this.getVectorToTargetSOTM0();
        double targetDistanceSOTM = Math.abs(Math.hypot(adjustedTargetGoal.getX(), adjustedTargetGoal.getY()));
        SmartDashboard.putNumber("Target/dist-sotm-p0", targetDistanceSOTM);   
        return targetDistanceSOTM;
    }

    /**
     * Iteratively converge on SOTM distance for higher accuracy.
     * Recalculates time-of-flight and robot movement multiple times.
     */
    public double getTargetDistanceSOTMConverged() {
        double dist = getTargetDistance();
        for (int i = 0; i < 3; i++) {
            double tof = getTimeOfFlight(dist);
            ChassisSpeeds speeds = s_Swerve.getSpeeds();
            double robotVx = speeds.vxMetersPerSecond;
            double robotVy = speeds.vyMetersPerSecond;
            Translation2d robotMovement = new Translation2d(robotVx * tof, robotVy * tof);
            Translation2d adjustedTarget = this.getVectorToTarget().minus(robotMovement);
            dist = Math.abs(Math.hypot(adjustedTarget.getX(), adjustedTarget.getY()));
        }
        SmartDashboard.putNumber("Target/dist-sotm-converged", dist);
        return dist;
    }


    // Define a simple record for our data points
    private record InterpolationPoint(double distance, double speed, double timeOfFlight) {}

    // The Look Up Table (LUT) - keep this sorted by distance!
    private final List<InterpolationPoint> SHOOTER_LUT = List.of(
        new InterpolationPoint(1.0, 39.0, 0.6),
        new InterpolationPoint(1.5, 42.5, 0.65),
        new InterpolationPoint(2.0, 45.0, 0.7),
        new InterpolationPoint(3.0, 52.0, 0.75), // jason timed by hand, yay
        new InterpolationPoint(4.0, 56.0, 0.9),
        new InterpolationPoint(5.0, 58.0, 1.0),
        new InterpolationPoint(6.0, 65.0, 1.1),
        new InterpolationPoint(8.0, 80.0, 1.2)
    );

    // lookup dist in the SHOOTER_LUT and return the [lower,upper] interpolation points.
    // if the dist is outside the bounds of the LUT, return the closest interpolation point.
    // assumes lut sorted by distance ascending.
    public InterpolationPoint[] getInterpolationPoints(double dist) {
        if (dist <= SHOOTER_LUT.get(0).distance()) return new InterpolationPoint[]{SHOOTER_LUT.get(0)};
        for (int i = 0; i < SHOOTER_LUT.size() - 1; i++) {
            InterpolationPoint lower = SHOOTER_LUT.get(i);
            InterpolationPoint upper = SHOOTER_LUT.get(i + 1);
            if (dist <= upper.distance()) {
                return new InterpolationPoint[]{lower, upper};
            }
        }
        return new InterpolationPoint[]{SHOOTER_LUT.get(SHOOTER_LUT.size() - 1)};
    }

    public double getTimeOfFlight(double dist) {
        InterpolationPoint[] points = getInterpolationPoints(dist);
        if (points.length == 1) return points[0].timeOfFlight();
        InterpolationPoint lower = points[0], upper = points[1];
        // Linear Interpolation formula: y = y1 + (x - x1) * ((y2 - y1) / (x2 - x1))
        double t = (dist - lower.distance()) / (upper.distance() - lower.distance());
        return lower.timeOfFlight() + t * (upper.timeOfFlight() - lower.timeOfFlight());
    }

    public double getTargetSpeed(double dist) {
        InterpolationPoint[] points = getInterpolationPoints(dist);
        if (points.length == 1) return points[0].speed();
        InterpolationPoint lower = points[0], upper = points[1];
        // Linear Interpolation formula: y = y1 + (x - x1) * ((y2 - y1) / (x2 - x1))
        double t = (dist - lower.distance()) / (upper.distance() - lower.distance());
        return lower.speed() + t * (upper.speed() - lower.speed());
    }
    
    // if we call with no args then we get the default distance
    public double getTargetSpeed() {
        double dist = this.getTargetDistance();
        return getTargetSpeed(dist);
    }

    // ── Targeting ──────────────────────────────────────────────────────────────
    
    // Get turret angle based on the current targeting mode (BASIC, SOTM, or SOTM_CONVERGED).
    public double getTurretAngle() {
        Pose2d robotPose = getRelativeRobotPose();
        // Select the target vector based on targeting mode
        Translation2d toTarget = switch (targetingMode) {
            case BASIC -> this.getVectorToTarget();
            case SOTM -> this.getVectorToTargetSOTM0();
            case SOTM_CONVERGED -> {
                double dist = getTargetDistanceSOTMConverged();
                ChassisSpeeds speeds = s_Swerve.getSpeeds();
                double robotVx = speeds.vxMetersPerSecond;
                double robotVy = speeds.vyMetersPerSecond;
                double tof = getTimeOfFlight(dist);
                Translation2d robotMovement = new Translation2d(robotVx * tof, robotVy * tof);
                yield this.getVectorToTarget().plus(robotMovement.times(-1));
            }
        };
        
        // Compute turret angle from the target vector and robot heading
        double targetAngle = Math.atan2(toTarget.getY(), toTarget.getX()); // radians
        double turretAngle = targetAngle - robotPose.getRotation().getRadians();

        // we need to pre-rotate the angle by ~90-deg clockwise and then modulo the angle 
        // and then post-rotate the angle by ~90-deg counter-clockwise.
        turretAngle = turretAngle - Math.toRadians(85);
        turretAngle = MathUtil.angleModulus(turretAngle); // normalize to [-pi, pi]
        turretAngle = turretAngle + Math.toRadians(85);

        SmartDashboard.putNumber("Turret Angle", Math.toDegrees(turretAngle));        
        // important to return as degrees b/c other calculations were in radians!
        return Math.toDegrees(turretAngle);

    }
    
    public double getHoodAngle() {
        Pose2d robotPose = this.getRelativeRobotPose();
        // If we are near the trench then drop the hood to lowest position
        double safeDistance = Units.inchesToMeters(12); // 12 inches of buffer on either side of the trench
        double ourTrenchSafeStartX = Constants.Localization.trenchline - safeDistance;
        double ourTrenchSafeEndX = Constants.Localization.trenchline + safeDistance;
        double oppositeTrenchSafeStartX = Constants.Localization.oppositeTrenchline - safeDistance;
        double oppositeTrenchSafeEndX = Constants.Localization.oppositeTrenchline + safeDistance;

        // previously was 4.3 - 4.9 meters, but we are using more accurate field dimensions now.
        if ((robotPose.getX() > ourTrenchSafeStartX && robotPose.getX() < ourTrenchSafeEndX) ||
            (robotPose.getX() > oppositeTrenchSafeStartX && robotPose.getX() < oppositeTrenchSafeEndX)) {
            return Constants.MIN_HOOD_ANGLE;
        }

        // Select the target distance based on targeting mode
        double targetDistance = switch (targetingMode) {
            case BASIC -> this.getTargetDistance();
            case SOTM -> this.getTargetDistanceSOTM0();
            case SOTM_CONVERGED -> this.getTargetDistanceSOTMConverged();
        };

        // Compute hood angle from the target distance
        double hoodAngle = targetDistance * 60;
        SmartDashboard.putNumber("Hood Angle", hoodAngle);
        return Math.abs(MathUtil.clamp(hoodAngle, Constants.MIN_HOOD_ANGLE, Constants.MAX_HOOD_ANGLE));
    }

    /**
     * Calculate turret angle correction for robot yaw rate (angular velocity).
     * Compensates for robot rotation during time-of-flight.
     * Returns the angle adjustment in degrees.
     */
    public double getYawRateLeadCompensation() {
        double tof = getTimeOfFlight(getTargetDistance());
        double yawRateRadPerSec = s_Swerve.getYawRate();
        double yawRotationDuringFlight = Math.toDegrees(yawRateRadPerSec * tof);
        SmartDashboard.putNumber("Yaw Rate Lead Compensation", yawRotationDuringFlight);
        return -yawRotationDuringFlight;
    }

}
