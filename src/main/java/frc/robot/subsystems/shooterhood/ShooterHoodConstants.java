package frc.robot.subsystems.shooterhood;

public class ShooterHoodConstants {

    public static final ShooterHoodGains SimGains = new ShooterHoodGains(10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    public static final ShooterHoodGains TalonFXGains = new ShooterHoodGains(0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    
    record ShooterHoodGains(double kP, double kI, double kD, double kS, double kG, double kV, double kA) {
        public ShooterHoodGains {
            if (kP < 0 || kI < 0 || kD < 0 || kS < 0 || kV < 0 || kA < 0 || kG < 0) {
                throw new IllegalArgumentException("Gains must be non-negative");
            }
        }
    }
}
