package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.MethodOrderer;
import org.junit.jupiter.api.Order;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestMethodOrder;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Boots the whole robot in simulation (real Robot class, simulated CTRE devices and mechanism physics)
 * and holds the shoot button in teleop. Proves the full chain works end to end: controller ->
 * Superstructure -> flywheel spins up -> simulated balls -> current spikes -> BallDetector counts them.
 *
 * Then runs a real PathPlanner auto and checks the simulated swerve modules carry the robot to the end.
 *
 * <p>CTRE's simulator runs in real time, so this test takes a few seconds.
 */
@TestMethodOrder(MethodOrderer.OrderAnnotation.class)
class RobotSimTest {
    private static Robot robot;

    @BeforeAll
    static void boot() {
        assertTrue(HAL.initialize(500, 0));
        robot = new Robot();
    }

    @AfterAll
    static void shutdown() {
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();
    }

    private static void loop(int iterations) throws InterruptedException {
        for (int i = 0; i < iterations; i++) {
            robot.robotPeriodic();
            robot.teleopPeriodic();
            robot.simulationPeriodic();
            Thread.sleep(20);
        }
    }

    /** Robot pose from the "Field" widget: {x m, y m, heading deg}. */
    private static double[] fieldPose() {
        SmartDashboard.updateValues();
        return NetworkTableInstance.getDefault().getTable("SmartDashboard").getSubTable("Field")
                .getEntry("Robot").getDoubleArray(new double[3]);
    }

    @Test
    @Order(1)
    void holdingShootSpinsUpAndShootsBalls() throws InterruptedException {
        DriverStationSim.setAutonomous(false);
        DriverStationSim.setTest(false);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        robot.teleopInit();

        XboxControllerSim pad = new XboxControllerSim(0);
        pad.setRightBumperButton(true);
        pad.notifyNewData();

        boolean sawReady = false;
        double balls = 0;
        for (int i = 0; i < 20 && balls < 2; i++) { // up to ~6 s
            loop(15);
            sawReady |= SmartDashboard.getBoolean("Driver/Ready", false);
            balls = SmartDashboard.getNumber("Driver/Balls Shot", 0);
        }
        assertTrue(sawReady, "flywheel should reach speed with the turret on target");
        assertTrue(balls >= 2, "ball detector should count simulated balls, counted " + balls);

        pad.setRightBumperButton(false);
        pad.notifyNewData();
        loop(5);
        assertTrue(SmartDashboard.getBoolean("Driver/Healthy", false)
                || !SmartDashboard.getString("Driver/Robot Health", "").contains("NOT ON CAN"),
                "no simulated device should be missing from CAN: " + SmartDashboard.getString("Driver/Robot Health", ""));
    }

    @Test
    @Order(2)
    void autoDrivesThePathWithTheSwerveModules() throws InterruptedException {
        DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
        DriverStationSim.setAutonomous(true);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        // "Outpost Start - Backup": (3.5, 0.6) heading 0 -> (1.614, 1.429) heading 30, about 2 s.
        NetworkTableInstance.getDefault().getTable("SmartDashboard").getSubTable("Auto Chooser")
                .getEntry("selected").setString("Outpost Backup and Shoot");
        SmartDashboard.updateValues();
        robot.autonomousInit();

        // DogLog publishes to NetworkTables under /Robot/ (MotorTelemetry -> Motors/<name>/...).
        var flDriveRpm = NetworkTableInstance.getDefault().getDoubleTopic("/Robot/Motors/FL drive/VelocityRPM")
                .subscribe(0);
        double maxRpm = 0;
        for (int i = 0; i < 200; i++) { // ~4 s
            robot.robotPeriodic();
            robot.simulationPeriodic();
            Thread.sleep(20);
            maxRpm = Math.max(maxRpm, Math.abs(flDriveRpm.get()));
        }
        // ~2 m/s along the path is ~2000 motor rpm; the old sim moved the pose but never the motors.
        assertTrue(maxRpm > 1000, "FL drive motor should spin in sim, max rpm " + maxRpm);
        double[] pose = fieldPose();
        assertEquals(1.614, pose[0], 0.25, "x at the end of the path");
        assertEquals(1.429, pose[1], 0.25, "y at the end of the path");
        assertEquals(30, pose[2], 8, "heading at the end of the path");
    }
}
