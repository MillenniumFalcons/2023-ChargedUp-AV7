package team3647.frc2023.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.DoubleSupplier;
import team3647.frc2023.subsystems.Pivot;

public class PivotCommands {

    public Command openloop(DoubleSupplier demand) {
        return Commands.run(() -> pivot.setOpenLoop(demand.getAsDouble()), this.pivot);
    }

    private final Pivot pivot;

    public PivotCommands(Pivot pivot) {
        this.pivot = pivot;
    }
}
