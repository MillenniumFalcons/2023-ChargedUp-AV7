package team3647.frc2023.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.DoubleSupplier;
import team3647.frc2023.subsystems.Pivot;

public class PivotCommands {

    public Command openloop(DoubleSupplier demand) {
        return Commands.run(() -> pivot.setOpenloop(demand.getAsDouble()), this.pivot);
    }

    public Command setAngle(double setpoint) {
        return Commands.run(() -> pivot.setAngle(setpoint), pivot)
                .until(() -> Math.abs(pivot.getAngle() - setpoint) < 0.05);
    }

    public Command setAngle(DoubleSupplier setpoint) {
        return Commands.run(() -> pivot.setAngle(setpoint.getAsDouble()), pivot)
                .until(() -> Math.abs(pivot.getAngle() - setpoint.getAsDouble()) < 0.05);
    }

    private final Pivot pivot;

    public PivotCommands(Pivot pivot) {
        this.pivot = pivot;
    }
}