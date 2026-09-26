package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Optional;

import org.junit.jupiter.api.Test;

import edu.wpi.first.wpilibj.DriverStation.Alliance;

class HubScheduleTest {
    private static final Optional<Alliance> RED = Optional.of(Alliance.Red);
    private static final Optional<Alliance> BLUE = Optional.of(Alliance.Blue);

    private static HubSchedule.Status teleop(Optional<Alliance> us, double matchTime, String data) {
        return HubSchedule.status(us, false, true, matchTime, data);
    }

    @Test
    void bothHubsActiveInAutoTransitionAndEndgame() {
        assertTrue(HubSchedule.status(RED, true, false, 15, "").active());
        assertTrue(teleop(RED, 135, "R").active()); // transition shift
        assertTrue(teleop(RED, 20, "R").active()); // endgame
        assertTrue(teleop(BLUE, 20, "R").active());
    }

    @Test
    void theAllianceNamedInGameDataGoesInactiveFirst() {
        // 'R': red inactive in shifts 1 and 3, active in 2 and 4. Blue is the opposite.
        double[] shiftMidpoints = {117, 92, 67, 42};
        boolean[] redActive = {false, true, false, true};
        for (int i = 0; i < 4; i++) {
            assertEquals(redActive[i], teleop(RED, shiftMidpoints[i], "R").active(), "red shift " + (i + 1));
            assertEquals(!redActive[i], teleop(BLUE, shiftMidpoints[i], "R").active(), "blue shift " + (i + 1));
        }
        assertTrue(teleop(RED, 117, "B").active(), "'B' means blue goes inactive first");
    }

    @Test
    void countsDownToTheNextChange() {
        HubSchedule.Status s = teleop(RED, 120, "R"); // shift 1, red inactive until 105
        assertFalse(s.active());
        assertEquals(15, s.secondsUntilChange(), 1e-9);
        assertEquals(10, teleop(RED, 140, "R").secondsUntilChange(), 1e-9, "transition -> shift 1 at 130");
        HubSchedule.Status last = teleop(RED, 40, "R"); // shift 4, red active, stays active into endgame
        assertTrue(last.active());
        assertTrue(Double.isInfinite(last.secondsUntilChange()));
    }

    @Test
    void unknownWithoutGameDataOrAllianceAssumesActive() {
        HubSchedule.Status noData = teleop(RED, 100, "");
        assertTrue(noData.active());
        assertFalse(noData.known());
        assertFalse(teleop(Optional.empty(), 100, "R").known());
        assertFalse(teleop(RED, 100, "X").known());
    }

    @Test
    void labelIsReadable() {
        assertEquals("INACTIVE 15s", teleop(RED, 120, "R").label());
        assertEquals("ACTIVE", teleop(RED, 20, "R").label());
    }
}
