package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;

/**
 * One place that knows every CTRE device and can say what's wrong with the robot in plain English.
 * Devices register themselves when they're created (see {@link Talons#create}); cameras add a check.
 */
public final class Health {
    private Health() {}

    private record Device(String name, ParentDevice device) {}

    private static final List<Device> DEVICES = new ArrayList<>();
    /** Extra checks: return null when fine, or a short problem description. */
    private static final List<Supplier<String>> CHECKS = new ArrayList<>();

    public static void register(String name, ParentDevice device) {
        DEVICES.add(new Device(name, device));
    }

    public static void addCheck(Supplier<String> check) {
        CHECKS.add(check);
    }

    /** @return every current problem; empty when healthy */
    public static List<String> problems() {
        List<String> problems = new ArrayList<>();
        for (Device d : DEVICES) {
            String issue = deviceIssue(d.device());
            if (issue != null) problems.add(d.name() + ": " + issue);
        }
        for (Supplier<String> check : CHECKS) {
            String issue = check.get();
            if (issue != null) problems.add(issue);
        }
        for (CANBus bus : List.of(Constants.CAN.RIO, Constants.CAN.DRIVETRAIN)) {
            var status = bus.getStatus();
            if (!status.Status.isOK()) problems.add("CAN bus '" + bus.getName() + "': " + status.Status.getName());
            else if (status.BusUtilization > 0.9) problems.add(String.format("CAN bus '%s' %.0f%% busy", bus.getName(), status.BusUtilization * 100));
        }
        double volts = RobotController.getBatteryVoltage();
        if (volts < Constants.Health.LOW_BATTERY_V) problems.add(String.format("LOW BATTERY %.1f V", volts));
        if (RobotController.isBrownedOut()) problems.add("BROWNED OUT");
        return problems;
    }

    private static String deviceIssue(ParentDevice device) {
        if (!device.isConnected()) return "NOT ON CAN";
        if (device instanceof TalonFX m) {
            if (m.getFault_Hardware().getValue()) return "hardware fault";
            if (m.getFault_UnlicensedFeatureInUse().getValue()) return "unlicensed feature (Phoenix Pro)";
            if (m.getFault_UsingFusedCANcoderWhileUnlicensed().getValue()) return "fused CANcoder needs Pro";
            if (m.getFault_BridgeBrownout().getValue()) return "browned out";
            if (m.getFault_DeviceTemp().getValue() || m.getFault_ProcTemp().getValue()) return "OVERHEATING";
            if (m.getStickyFault_BootDuringEnable().getValue()) return "rebooted while enabled (check power)";
        } else if (device instanceof CANcoder e) {
            if (e.getFault_BadMagnet().getValue()) return "BAD MAGNET";
            if (e.getFault_Hardware().getValue()) return "hardware fault";
        } else if (device instanceof Pigeon2 g) {
            if (g.getFault_Hardware().getValue()) return "hardware fault";
        }
        return null;
    }
}
