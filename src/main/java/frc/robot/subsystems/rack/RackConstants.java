package frc.robot.subsystems.rack;

public class RackConstants {

    public static final RackGains SimGains = new RackGains(10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    public static final RackGains TalonFXGains = new RackGains(0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    
    record RackGains(double kP, double kI, double kD, double kS, double kG, double kV, double kA) {
        public RackGains {
            //if (kP < 0 || kI < 0 || kD < 0 || kS < 0 || kV < 0 || kA < 0 || kG < 0) {
            //    throw new IllegalArgumentException("Gains must be non-negative");
            //}
        }
    }
}
