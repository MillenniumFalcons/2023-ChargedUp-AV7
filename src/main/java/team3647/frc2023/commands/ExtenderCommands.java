package team3647.frc2023.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.DoubleSupplier;
import team3647.frc2023.subsystems.Extender;

public class ExtenderCommands {

    public Command openloop(DoubleSupplier demand) {
        return Commands.run(() -> extender.setOpenloop(demand.getAsDouble()), this.extender);
    }

    public Command openLoopSlow(DoubleSupplier demand) {
        return Commands.run(() -> extender.setOpenloop(demand.getAsDouble() * 0.5), this.extender);
    }

    public Command length(double length) {
        return Commands.run(() -> extender.setLengthMeters(length), extender);
    }

    private final Extender extender;

    public ExtenderCommands(Extender extender) {
        this.extender = extender;
    }
}
