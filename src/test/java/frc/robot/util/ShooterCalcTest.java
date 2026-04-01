import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.MathUtil;

import frc.robot.util.ShooterCalc;
import frc.robot.subsystems.swerve.Swerve;

import org.junit.jupiter.api.*;

class ShooterCalcTest {
    private ShooterCalc shooterCalcBasic, shooterCalcSOTM, shooterCalcSOTMConverged;
    private static Swerve swerve;

    @BeforeAll
    static void setupOnce() {
        assertTrue(HAL.initialize(500, 0));
        // Create a single Swerve instance shared across all tests
        // This avoids re-configuring PathPlanner's AutoBuilder multiple times
        swerve = new Swerve();
    }

    @BeforeEach
    void setup() {
        // Create fresh ShooterCalc instances with different targeting modes
        // These are lightweight and don't trigger AutoBuilder configuration
        swerve.setPose(new Pose2d(0, 0, new Rotation2d(0))); // reset pose before each test
        swerve.driveRobotRelativeAuto(new ChassisSpeeds(0, 0, 0)); // reset velocity before each test
        shooterCalcBasic = new ShooterCalc(swerve, ShooterCalc.TargetingMode.BASIC);
        shooterCalcSOTM = new ShooterCalc(swerve, ShooterCalc.TargetingMode.SOTM);
        shooterCalcSOTMConverged = new ShooterCalc(swerve, ShooterCalc.TargetingMode.SOTM_CONVERGED);
    }

    /**
     * Test that all three targeting modes produce identical results when robot velocity is zero.
     * This verifies that SOTM and SOTM_CONVERGED degrade gracefully to BASIC when not moving.
     * Since the robot starts at rest in simulation, all three modes should match immediately.
     */
    @Test
    void testAllModesMatchAtZeroVelocity() {
        // When the swerve is at rest (which it is at the start), all three modes should match
        double turretBasic = shooterCalcBasic.getTurretAngle();
        double turretSOTM = shooterCalcSOTM.getTurretAngle();
        double turretSOTMConverged = shooterCalcSOTMConverged.getTurretAngle();

        // All turret angles should match at zero velocity (within floating-point tolerance)
        assertEquals(turretBasic, turretSOTM, 0.5, "Turret angles should match at zero velocity");
        assertEquals(turretBasic, turretSOTMConverged, 0.5, "Turret angles should match at zero velocity");

        double hoodBasic = shooterCalcBasic.getHoodAngle();
        double hoodSOTM = shooterCalcSOTM.getHoodAngle();
        double hoodSOTMConverged = shooterCalcSOTMConverged.getHoodAngle();

        // All hood angles should match
        assertEquals(hoodBasic, hoodSOTM, 0.5, "Hood angles should match at zero velocity");
        assertEquals(hoodBasic, hoodSOTMConverged, 0.5, "Hood angles should match at zero velocity");
    }

    /**
     * Test that SOTM produces different results than BASIC when robot is moving.
     * Robot is positioned at (1, 2) and moving at vx=0, vy=1 m/s.
     */
    @Test
    void testSOTMDiffersFromBasicWhenMoving1() {
        // Setup: Position robot at (1, 2) with heading 0
        Pose2d testPose = new Pose2d(1.0, 2.0, new Rotation2d(0));
        swerve.setPose(testPose);
        
        // Validate swerve getPose() returns the commanded pose (works in simulation)
        var currentPose = swerve.getPose();
        assertEquals(1.0, currentPose.getX(), 0.1, "x should be approximately 1 m");
        assertEquals(2.0, currentPose.getY(), 0.1, "y should be approximately 2 m");
        assertEquals(0.0, currentPose.getRotation().getDegrees(), 0.1, "heading should be approximately 0°");
        
        // Set velocity: moving forward in Y direction (vx=0, vy=1 m/s, omega=0)
        ChassisSpeeds velocity = new ChassisSpeeds(0.0, 1.0, 0.0);
        swerve.driveRobotRelative(velocity, true); // true for open-loop to set velocity directly without acceleration limits
        
        // Validate swerve getSpeeds() returns the commanded speed (works in simulation)
        var currentSpeeds = swerve.getSpeeds();
        assertEquals(0.0, currentSpeeds.vxMetersPerSecond, 0.1, "vx should be approximately 0 m/s");
        assertEquals(1.0, currentSpeeds.vyMetersPerSecond, 0.1, "vy should be approximately 1 m/s");
        assertEquals(0.0, currentSpeeds.omegaRadiansPerSecond, 0.1, "omega should be approximately 0 rad/s");

        // Create fresh instances with the new position/velocity state
        ShooterCalc basicCalc = new ShooterCalc(swerve, ShooterCalc.TargetingMode.BASIC);
        ShooterCalc sotmCalc = new ShooterCalc(swerve, ShooterCalc.TargetingMode.SOTM);
        
        // Get turret angles
        double turretBasic = basicCalc.getTurretAngle();
        double turretSOTM = sotmCalc.getTurretAngle();
        
        // Get hood angles
        double hoodBasic = basicCalc.getHoodAngle();
        double hoodSOTM = sotmCalc.getHoodAngle();
        
        // SOTM should produce different angles when robot is moving
        // (SOTM compensates for robot motion, so angles differ from static BASIC)
        assertNotEquals(turretBasic, turretSOTM, 0.1, "SOTM turret angle should differ from BASIC when robot is moving (vx=0, vy=1)");
        assertNotEquals(hoodBasic, hoodSOTM, 0.1, "SOTM hood angle should differ from BASIC when robot is moving (vx=0, vy=1)");
        
        assertEquals(turretBasic, -29.3, 1, "BASIC turret angle should be approximately -29.3 deg at (1, 2) with vx=0, vy=1");
        assertEquals(turretSOTM, -43.6, 1, "SOTM turret angle should be approximately -43.6 deg at (1, 2) with vx=0, vy=1");
        assertEquals(hoodBasic, 249.0, 1, "BASIC hood should be approximately 249.0 at (1, 2) with vx=0, vy=1");
        assertEquals(hoodSOTM, 300.0, 1, "SOTM hood should be approximately 300 at (1, 2) with vx=0, vy=1");
    }
    /**
     * Test that SOTM produces different results than BASIC when robot is moving.
     * Robot is positioned at (1, 2) and moving at vx=0, vy=1 m/s.
     */
    @Test
    void test1mToHub_45degOffset() {
        // Setup: Position robot 1m in front of hub (4.62, 4.034) with heading 45 degrees
        swerve.setPose(new Pose2d(3.62, 4.034, new Rotation2d(0.7854))); // 45 degrees in radians
        Pose2d currentPose = swerve.getPose();
        assertEquals(3.62, currentPose.getX(), 0.1, "x should be approximately 3.62 m");
        assertEquals(4.034, currentPose.getY(), 0.1, "y should be approximately 4.034 m");
        assertEquals(45.0, currentPose.getRotation().getDegrees(), 0.1, "heading should be approximately 45 deg");
        ShooterCalc basicCalc = new ShooterCalc(swerve, ShooterCalc.TargetingMode.BASIC);
        assertEquals(basicCalc.getTurretAngle(), 45, 1, "BASIC turret angle should be approximately 45 deg at (3.62, 4.034)");
        assertEquals(basicCalc.getHoodAngle(), 100.0, 1, "BASIC hood should be approximately 100 at (3.62, 4.034)");
    }
    @Test 
    void test_1p5mToHub_m30degOffset() {
        // Setup: Position robot 1.5m offset from the hub at 30 deg offset with heading 0 degrees
        double x = 4.62 - 0.75; // 1.5m from hub at 30 deg offset
        double y = 4.034 + 1.3; // 1.5m from hub at 30 deg offset
        swerve.setPose(new Pose2d(x, y, new Rotation2d(0))); // 1.5m from hub at 30 deg offset
        ShooterCalc basicCalc = new ShooterCalc(swerve, ShooterCalc.TargetingMode.BASIC);
        assertEquals(basicCalc.getTargetDistance(), 1.5, 0.1, "Target distance should be approximately 1.5 m");
        assertEquals(basicCalc.getVectorToTarget().getX(), -0.75, 0.1, "Vector to target X should be approximately 0.75 m");
        assertEquals(basicCalc.getVectorToTarget().getY(), 1.3, 0.1, "Vector to target Y should be approximately 1.3 m");

        Pose2d robotPose = basicCalc.getRelativeRobotPose();
        var toTarget = basicCalc.getVectorToTarget();
        double targetAngle = Math.toDegrees(Math.atan2(toTarget.getY(), toTarget.getX()));
        double turretAngle = targetAngle - robotPose.getRotation().getDegrees();
        turretAngle = MathUtil.inputModulus(turretAngle + 180, -185, 185);  
        var returnAngle = -(MathUtil.clamp(turretAngle, -185.0, 185.0));
        //assertEquals(returnAngle, -30.0,1, "Calculated turret angle should be approximately -30.0 deg at 1.5m away with 30 deg offset");
        //assertEquals(turretAngle, -30.0, 1, "Calculated turret angle should be approximately -30.0 deg at 1.5m away with 30 deg offset");
        assertEquals(targetAngle, 120.0, 1, "Calculated target angle should be approximately 120.0 deg at 1.5m away with 30 deg offset");
        assertEquals(robotPose.getRotation().getDegrees(), 0.0, 0.1, "Robot heading should be approximately 0 deg");

        //assertEquals(basicCalc.getTurretAngle(), Math.toDegrees(Math.atan2(1.3, -0.75)), 1, "BASIC turret angle should be approximately -30.0 deg and 1.5m away");
        assertEquals(basicCalc.getHoodAngle(), 120.0, 1, "BASIC hood should be approximately 120.0 at 1.5m away");
    }
    //@Test 
    // void dontTestJunk () {
    //     // Set velocity: moving forward in Y direction (vx=0, vy=1 m/s, omega=0)
    //     ChassisSpeeds velocity = new ChassisSpeeds(1.0, 1.0, 0.0);
    //     swerve.driveRobotRelative(velocity, true); // true for open-loop to set velocity directly without acceleration limits    
    //     // Validate swerve getSpeeds() returns the commanded speed (works in simulation)
    //     var currentSpeeds = swerve.getSpeeds();
    //     assertEquals(1.0, currentSpeeds.vxMetersPerSecond, 0.1, "vx should be approximately 1 m/s");
    //     assertEquals(1.0, currentSpeeds.vyMetersPerSecond, 0.1, "vy should be approximately 1 m/s");
    //     assertEquals(0.0, currentSpeeds.omegaRadiansPerSecond, 0.1, "omega should be approximately 0 rad/s");

    //     // Create fresh instances with the new position/velocity state
    //     ShooterCalc sotmCalc = new ShooterCalc(swerve, ShooterCalc.TargetingMode.SOTM);
        
    //     // Get turret angles
    //     double turretSOTM = sotmCalc.getTurretAngle();
        
    //     // Get hood angles
    //     double hoodSOTM = sotmCalc.getHoodAngle();
        
    //     // SOTM should produce different angles when robot is moving
    //     // (SOTM compensates for robot motion, so angles differ from static BASIC)
    //     assertNotEquals(turretBasic, turretSOTM, 0.1, "SOTM turret angle should differ from BASIC when robot is moving (vx=0, vy=1)");
    //     assertNotEquals(hoodBasic, hoodSOTM, 0.1, "SOTM hood angle should differ from BASIC when robot is moving (vx=0, vy=1)");
        
    //     assertEquals(turretSOTM, 18.5, 1, "SOTM turret angle should be approximately 18.5 deg at (3.62, 4.034) with vx=1, vy=1");
    //     assertEquals(hoodSOTM, 134.0, 1, "SOTM hood should be approximately 134.0 at (3.62, 4.034) with vx=1, vy=1");
    // }

    /**
     * Test that getTurretAngle produces a valid angle.
     */
    @Test
    void testTurretAngleIsValid() {
        double turretAngle = shooterCalcBasic.getTurretAngle();
        // Turret angle should be a real number
        assertFalse(Double.isNaN(turretAngle), "Turret angle should not be NaN");
        assertFalse(Double.isInfinite(turretAngle), "Turret angle should not be infinite");
    }

    /** Test that getHoodAngle produces a valid angle. */
    @Test
    void testHoodAngleIsValid() {
        double hoodAngle = shooterCalcBasic.getHoodAngle();
        // Hood angle should be a real number
        assertFalse(Double.isNaN(hoodAngle), "Hood angle should not be NaN");
        assertFalse(Double.isInfinite(hoodAngle), "Hood angle should not be infinite");        
        // Hood angle should be within a reasonable range (0-60 degrees is typical)
        assertTrue(hoodAngle >= 100 && hoodAngle <= 400, "Hood angle should be between 100 and 400 units, was: " + hoodAngle);
    }

    /** Test that yaw rate compensation produces valid values. */
    @Test
    void testYawRateCompensationIsValid() {
        double yawCompensation = shooterCalcSOTM.getYawRateLeadCompensation();
        // Yaw compensation should be a real number
        assertFalse(Double.isNaN(yawCompensation), "Yaw compensation should not be NaN");
        assertFalse(Double.isInfinite(yawCompensation), "Yaw compensation should not be infinite");
    }
}