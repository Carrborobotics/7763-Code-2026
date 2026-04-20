package frc.robot;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Pounds;

//import java.time.YearMonth;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.robot.subsystems.swerve.SwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.12;
    public static final double stickDeadbandDivider = 1.0; // 1 FOR COMP, 5 for test mode.
    
    public static final double RACK_GEARING = 9.0; // 9:1
    public static final double INTAKE_GEARING = 1.25; // 12t:15t = 1.25:1
    public static final double TURRET_ROTATION_GEARING = 12.0; // 12:1
    public static final double SHOOTER_HOOD_GEARING = 715.5; // 715.5:1 (wow)
    public static final double SHOOTER_GEARING = 1.0;
    public static final double FLOOR_GEARING = 1.0;
    public static final double shooterRPS = 100;
    
    public static final double RACK_EXTEND_POSITION = -32000.0; // was -34000
    public static final double RACK_RETRACT_POSITION = 0.0;
    
    public static final double MIN_HOOD_ANGLE = 100.0; // units, not really degrees
    public static final double MAX_HOOD_ANGLE = 300.0; // units, not really degrees (previously 400.0)

    public static final class Intake {
        public static final double FORWARD_SPEED = 0.9; // was 0.5, 0.6 moving to 0.9 with net
        public static final double REVERSE_SPEED = -0.2;
    }

    public static final class CANConstants {
        public static final int intakeId = 42;
        public static final int rackId = 41;
        public static final int rackId2 = 43;
        public static final int turretId = 21; // on drivetrain
        public static final int shooterLeft = 22;
        public static final int shooterRight = 23;
        public static final int kickerId = 24;
        public static final int shooterHoodId = 25;
        public static final int floorId = 30;
        /* CANBusses */
        public static final CANBus canBus = new CANBus("rio");
        public static final CANBus canBusDriveTrain = new CANBus("Drivetrain");
    }

    public static final class Localization {

        // Field Dimensions
        public static final double fieldWidth = FlippingUtil.fieldSizeY;
        public static final double fieldLength = FlippingUtil.fieldSizeX;

        // Robot Dimensions
        //public static final double robotFrameLength = Units.inchesToMeters(27);
        //public static final double bumperWidth = Units.inchesToMeters(3);

        // Field Waypoints
        public static final double fieldX = Units.inchesToMeters(650.12); // AndyMark
        public static final double fieldY = Units.inchesToMeters(316.64); // AndyMark
        public static final double trenchline = Units.inchesToMeters(181.56); // AndyMark
        public static final double yMidline = fieldY / 2.0;
        public static final Translation2d hubPosition = new Translation2d(trenchline, yMidline);
        public static final double oppositeTrenchline = fieldX - trenchline;

        
        // passing zone locations
        public static final double passTargetX = Units.inchesToMeters(90); // somewhat arbitrary, as the alliance zone is ~158"
        public static final double lowerPassTargetY = Units.inchesToMeters(50);
        public static final double upperPassTargetY = fieldY - Units.inchesToMeters(50);
        public static final Translation2d lowerPassTarget = new Translation2d(passTargetX, lowerPassTargetY);
        public static final Translation2d upperPassTarget = new Translation2d(passTargetX, upperPassTargetY);
    }

    public static final class Swerve {

        public static final int pigeonID = 10;
        public static final CANBus CanBus = new CANBus("Drivetrain");
        public static final boolean focEnabled = true; 
        public static final boolean isOnCANivore = true;

        public static final COTSTalonFXSwerveConstants chosenModule = 
                COTSTalonFXSwerveConstants.WCP.SwerveXFlipped
                        .KrakenX60(COTSTalonFXSwerveConstants.WCP.SwerveXFlipped.driveRatios.X3_10);

        /* Drivetrain Constants */
        /* Center to Center distance of left and right modules in meters. */
        public static final double trackWidth = Units.inchesToMeters(25); 

        /* Center to Center distance of front and rear module wheels in meters. */
        public static final double wheelBase = Units.inchesToMeters(25); 
        public static final double wheelCircumference = chosenModule.wheelCircumference;
        public static final double wheelRadiusMeters = chosenModule.wheelDiameter / 2.0 ;
        /*
         * Swerve Kinematics
         * No need to ever change this unless you are not doing a traditional
         * rectangular/square 4 module swerve
         */
        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 30;
        public static final int angleCurrentThreshold = 40;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = false;

        public static final int driveCurrentLimit = 60;
        public static final int driveCurrentThreshold = 60;
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /*
         * These values are used by the          * loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc
         */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.12; // 0.12 FOR COMP
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = 0.32; // TODO: This must be tuned to specific robot
        public static final double driveKV = 1.51;
        public static final double driveKA = 0.27;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4.5; // 4.5 FOR COMP, 0.5 for test mode
        public static final double maxSpeedSOTM = 0.5; 
        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0; //10 FOR COMP, 2.0 for test mode
        public static final double maxAngularVelocitySOTM = 2.0;
        /*
         * These are theorectial values to start with, tune after
         * Kraken FOC-DIS (L1.0): ft/s = 12.9 | m/s = 3.93192
         * Kraken FOC-ENB (L1.0): ft/s = 12.4 | m/s = 3.77952
         * Kraken FOC-DIS (L1.5): ft/s = 14.2 | m/s = 4.32816
         * Kraken FOC-ENB (L1.5): ft/s = 14.2 | m/s = 4.32816
         * Kraken FOC-DIS (L2.0): ft/s = 15.5 | m/s = 4.7244
         * Kraken FOC-ENB (L2.0): ft/s = 15.0 | m/s = 4.572
         * Kraken FOC-DIS (L2.5): ft/s = 17.7 | m/s = 5.39496
         * Kraken FOC-ENB (L2.5): ft/s = 17.1 | m/s = 5.21208
         * Kraken FOC-DIS (L3.0): ft/s = 17.1 | m/s = 5.21208
         * Kraken FOC-ENB (L3.0): ft/s = 16.5 | m/s = 5.0292
         * Kraken FOC-DIS (L3.5): ft/s = 19.5 | m/s = 5.9436
         * Kraken FOC-ENB (L3.5): ft/s = 18.9 | m/s = 5.76072
         * Kraken FOC-DIS (L4.0): ft/s = 20.4 | m/s = 6.21792
         * Kraken FOC-ENB (L4.0): ft/s = 19.7 | m/s = 6.00456
         */


        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { 
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 15;
            //public static final Rotation2d angleOffset = Rotation2d.fromDegrees(90.0);
            private static final Rotation2d angleOffset = Rotation2d.fromRotations(0.285400390625);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 12;
            //public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-135.0);
            private static final Rotation2d angleOffset = Rotation2d.fromRotations(0.608642578125);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 { 
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 13;
            //public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-90.0);
            private static final Rotation2d angleOffset = Rotation2d.fromRotations(0.748046875);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { 
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 14;
            //public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-90.0);//180
            private static final Rotation2d angleOffset = Rotation2d.fromRotations(0.720703125);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }
    }

    public static final class AutoConstants { 
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;

        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class VisionConstants {
        public static final double kCameraRangeScaler = 0.1;
        public static final double kCameraAimScaler = 0.1;
        public static final double kCamHeight = 0.41;
        public static final double kTagHeight = 1.27;
        public static final double kCamPitch = Math.PI / 4; // ~45 degrees (pi/4 rad)
        public static final String kCameraNameC = "Arducam_OV9281_USB_Camera";
        public static final String kCameraNameR = "Arducam_OV9782_USB_Camera";

        public static final Transform3d kRobotToCamC = new Transform3d(
            new Translation3d(
                Units.inchesToMeters(13), 
                Units.inchesToMeters(1.75),
                Units.inchesToMeters(12)
            ), // X and Y were swapped?
            new Rotation3d(Units.degreesToRadians(0.0), Units.degreesToRadians(25), Units.degreesToRadians(0)));

        public static final Transform3d kRobotToCamR = new Transform3d(
            new Translation3d(
                Units.inchesToMeters(11.25), // x fwd
                Units.inchesToMeters(11.5), // y <-> over
                Units.inchesToMeters(12.25) // z=height
            ), // X and Y were swapped?
            new Rotation3d(Units.degreesToRadians(0.0), Units.degreesToRadians(25), Units.degreesToRadians(270)));


 //public static final Transform3d kRobotToCam = new Transform3d(
   //         new Translation3d(Units.inchesToMeters(9.15), Units.inchesToMeters(0.0), Units.inchesToMeters(7.25)), // X and Y were swapped?
     //             new Rotation3d(Units.degreesToRadians(0.0), Units.degreesToRadians(-10), Units.degreesToRadians(0)));

        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    }


    public static final class PathPlanner {
        public static final RobotConfig robotConfig = new RobotConfig(
            Mass.ofRelativeUnits(138, Pounds),
            MomentOfInertia.ofRelativeUnits(7.0, KilogramSquareMeters),
            new ModuleConfig(
                Swerve.wheelCircumference / (Math.PI * 2.0),
                Swerve.maxSpeed * 0.95, // Leave a little headroom for inefficiencies
                1.916, // 3847 Spectrum Vex GripLock v2 CoF
                DCMotor.getKrakenX60Foc(1),
                Swerve.chosenModule.driveGearRatio,
                Swerve.driveCurrentLimit,
                1),
            new Translation2d(Swerve.wheelBase / 2.0, Swerve.trackWidth / 2.0),
            new Translation2d(Swerve.wheelBase / 2.0, -Swerve.trackWidth / 2.0),
            new Translation2d(-Swerve.wheelBase / 2.0, Swerve.trackWidth / 2.0),
            new Translation2d(-Swerve.wheelBase / 2.0, -Swerve.trackWidth / 2.0));
    }

}
