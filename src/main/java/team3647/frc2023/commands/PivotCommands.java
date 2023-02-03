package team3647.frc2023.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import java.util.function.DoubleSupplier;
import team3647.frc2023.subsystems.Pivot;

public class PivotCommands {

    public Command openloop(DoubleSupplier demand) {
        return Commands.run(() -> pivot.setOpenloop(demand.getAsDouble()), this.pivot);
    }

    public Command setAngle(DoubleSupplier setpoint) {
        double degSetpoint = setpoint.getAsDouble();
        System.out.println(degSetpoint);
        return new FunctionalCommand(
                () -> {},
                () -> pivot.setAngle(setpoint.getAsDouble(), 0),
                interrupted -> {},
                () -> Math.abs(pivot.getAngle() - degSetpoint) < 0.05,
                pivot);
    }

    private final Pivot pivot;

    public PivotCommands(Pivot pivot) {
        this.pivot = pivot;
    }
}
