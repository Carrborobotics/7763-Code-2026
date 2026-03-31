import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import frc.robot.util.ShooterCalc;
import frc.robot.subsystems.swerve.Swerve;

import org.junit.jupiter.api.*;

class ShooterCalcTest {
    private ShooterCalc shooterCalcBasic, shooterCalcSOTM, shooterCalcSOTMConverged;
    private Swerve swerve;

    @BeforeEach
    void setup() {
        assertTrue(HAL.initialize(500, 0));
        // Create a real Swerve instance for testing
        // This allows us to test the actual integration with the subsystem
        swerve = new Swerve();
        // Create instances with different targeting modes
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
        assertTrue(hoodAngle >= 0 && hoodAngle <= 90, "Hood angle should be between 0 and 90 degrees");
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