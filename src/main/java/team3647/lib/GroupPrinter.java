package team3647.lib;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.HashMap;
import java.util.Map;
import java.util.Objects;
import java.util.function.Supplier;

/** Prints to SmartDashboard every loop */
public class GroupPrinter implements Subsystem {
    private static final GroupPrinter INSTANCE = new GroupPrinter();

    public static GroupPrinter getInstance() {
        return INSTANCE;
    }

    private final Map<String, Supplier<Double>> doublePrint = new HashMap<>();
    private final Map<String, Supplier<String>> stringPrint = new HashMap<>();
    private final Map<String, Supplier<Boolean>> boolPrint = new HashMap<>();
    private final Map<String, Supplier<Pose2d>> posePrint = new HashMap<>();
    private final Field2d field = new Field2d();

    private GroupPrinter() {}

    public void addDouble(String key, Supplier<Double> func) {
        Objects.requireNonNull(key);
        Objects.requireNonNull(func);
        doublePrint.put(key, func);
        SmartDashboard.putNumber(key, func.get());
    }

    public void addString(String key, Supplier<String> func) {
        Objects.requireNonNull(key);
        Objects.requireNonNull(func);
        stringPrint.put(key, func);
        SmartDashboard.putString(key, func.get());
    }

    public void addBoolean(String key, Supplier<Boolean> func) {
        Objects.requireNonNull(key);
        Objects.requireNonNull(func);
        boolPrint.put(key, func);
        SmartDashboard.putBoolean(key, func.get());
    }

    public void addPose(String key, Supplier<Pose2d> func) {
        Objects.requireNonNull(key);
        Objects.requireNonNull(func);
        if (posePrint.isEmpty()) {
            SmartDashboard.putData(field);
        }
        posePrint.put(key, func);
    }

    @Override
    public void periodic() {
        for (var entry : doublePrint.entrySet()) {
            SmartDashboard.putNumber(entry.getKey(), entry.getValue().get());
        }
        for (var entry : stringPrint.entrySet()) {
            SmartDashboard.putString(entry.getKey(), entry.getValue().get());
        }
        for (var entry : boolPrint.entrySet()) {
            SmartDashboard.putBoolean(entry.getKey(), entry.getValue().get());
        }
        for (var entry : posePrint.entrySet()) {
            Pose2d pose = entry.getValue().get();
            if (pose == null) {
                continue;
            }
            field.getObject(entry.getKey()).setPose(pose);
        }
    }

    public Field2d getField() {
        return field;
    }
}
