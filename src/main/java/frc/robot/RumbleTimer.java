package frc.robot;

/** Short controller-rumble pulses. Pure logic: Robot reads {@link #strength} and sends it to the controller. */
public final class RumbleTimer {
    private double untilS = Double.NEGATIVE_INFINITY;
    private double strength = 0;

    /** Start a pulse. A stronger pulse replaces a weaker one that is still running. */
    public void pulse(double nowS, double pulseStrength, double durationS) {
        if (nowS >= untilS || pulseStrength >= strength) {
            strength = pulseStrength;
            untilS = nowS + durationS;
        }
    }

    public double strength(double nowS) {
        return nowS < untilS ? strength : 0;
    }
}
