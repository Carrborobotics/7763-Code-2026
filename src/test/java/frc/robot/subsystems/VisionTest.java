package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

/** The match-view tag/mount values: the sign conventions are easy to get backwards. */
class VisionTest {
    @Test
    void tagOnTheLeftHasPositiveYaw() {
        assertEquals(45, Vision.tagYawDeg(new Transform3d(2, 2, 0.5, new Rotation3d())), 1e-9);
        assertEquals(-45, Vision.tagYawDeg(new Transform3d(2, -2, 0.5, new Rotation3d())), 1e-9);
        assertEquals(0, Vision.tagYawDeg(new Transform3d(3, 0, 1, new Rotation3d())), 1e-9);
    }

    @Test
    void tagDistanceIgnoresHeight() {
        assertEquals(5, Vision.tagFloorDistanceM(new Transform3d(3, 4, 2, new Rotation3d())), 1e-9);
    }

    @Test
    void mountArrayIsMetersThenDegrees() {
        var mount = new Transform3d(0.3, -0.1, 0.25,
                new Rotation3d(0, Units.degreesToRadians(25), Units.degreesToRadians(90)));
        assertArrayEquals(new double[] {0.3, -0.1, 0.25, 0, 25, 90}, Vision.mountArray(mount), 1e-9);
    }
}
