package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class RumbleTimerTest {
    @Test
    void pulseEndsOnTime() {
        RumbleTimer r = new RumbleTimer();
        assertEquals(0, r.strength(0));
        r.pulse(1.0, 0.5, 0.2);
        assertEquals(0.5, r.strength(1.1));
        assertEquals(0, r.strength(1.25));
    }

    @Test
    void weakPulseDoesNotCutOffAStrongOne() {
        RumbleTimer r = new RumbleTimer();
        r.pulse(0, 0.8, 0.4);
        r.pulse(0.1, 0.3, 0.1);
        assertEquals(0.8, r.strength(0.3));
    }
}
