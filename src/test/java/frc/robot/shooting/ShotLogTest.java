package frc.robot.shooting;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;

import org.junit.jupiter.api.Test;

class ShotLogTest {
    private static List<String> shoot(ShotLog log, double t) {
        return log.record(t, 100, "SCORING", 3.0, 52, 51, 180, 0.5, 0.2, 0);
    }

    @Test
    void marksTheWholeVolley() {
        ShotLog log = new ShotLog();
        shoot(log, 0.0);
        shoot(log, 0.3);
        shoot(log, 0.6);
        assertEquals(3, log.unmarkedCount());
        List<String> lines = log.mark(true);
        assertEquals(3, lines.size());
        assertTrue(lines.get(0).startsWith("1,"));
        assertTrue(lines.get(2).endsWith(",made"));
        assertEquals(0, log.unmarkedCount());
        assertEquals(3, log.madeCount());
    }

    @Test
    void unmarkedVolleyIsWrittenAsUnknownWhenTheNextOneStarts() {
        ShotLog log = new ShotLog();
        shoot(log, 0.0);
        shoot(log, 0.3);
        List<String> finished = shoot(log, 0.3 + ShotLog.VOLLEY_GAP_S + 1);
        assertEquals(2, finished.size());
        assertTrue(finished.get(1).endsWith(",unknown"));
        assertEquals(1, log.unmarkedCount());
    }

    @Test
    void csvHasOneColumnPerHeaderField() {
        ShotLog log = new ShotLog();
        shoot(log, 0);
        String line = log.mark(false).get(0);
        assertEquals(ShotLog.CSV_HEADER.split(",").length, line.split(",").length);
        assertEquals(1, log.missedCount());
    }
}
