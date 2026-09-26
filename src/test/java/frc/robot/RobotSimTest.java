package frc.robot;

import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Boots the whole robot in simulation (real Robot class, simulated CTRE devices and mechanism physics)
 * and holds the shoot button in teleop. Proves the full chain works end to end: controller ->
 * Superstructure -> flywheel spins up -> simulated balls -> current spikes -> BallDetector counts them.
 *
 * <p>CTRE's simulator runs in real time, so this test takes a few seconds.
 */
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

    @Test
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
}
