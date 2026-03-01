package frc.robot.subsystems.turret;

public class TurretConstants {

    public static final TurretGains SimGains = new TurretGains(10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    public static final TurretGains TalonFXGains = new TurretGains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    
    record TurretGains(double kP, double kI, double kD, double kS, double kG, double kV, double kA) {
        public TurretGains {
            if (kP < 0 || kI < 0 || kD < 0 || kS < 0 || kV < 0 || kA < 0 || kG < 0) {
                throw new IllegalArgumentException("Gains must be non-negative");
            }
        }
    }
}
