package team3647.frc2023.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.Set;
import java.util.function.DoubleSupplier;
import team3647.frc2023.constants.ExtenderConstants;
import team3647.frc2023.subsystems.Extender;

public class ExtenderCommands {

    public Command openloop(DoubleSupplier demand) {
        return Commands.run(() -> extender.setOpenloop(demand.getAsDouble()*0.5), this.extender);
    }

    public Command openLoopSlow(DoubleSupplier demand) {
        return Commands.run(() -> extender.setOpenloop(demand.getAsDouble() * 0.25), this.extender);
    }

    public Command length(DoubleSupplier length) {
        return Commands.run(() -> extender.setLengthMeters(length.getAsDouble()), extender)
                .until(() -> Math.abs(extender.getPosition() - length.getAsDouble()) < 5000);
    }

    public Command stow() {
        return length(() -> ExtenderConstants.kMinimumPositionTicks)
                .andThen(Commands.run(() -> {}, extender));
    }

    public Command holdPositionAtCall() {
        return new Command() {
            double lengthAtStart = ExtenderConstants.kMinimumPositionTicks;
            Set<Subsystem> reqs = Set.of(extender);

            @Override
            public void initialize() {
                lengthAtStart = extender.getPosition();
            }

            @Override
            public void execute() {
                extender.setLengthMeters(lengthAtStart);
            }

            @Override
            public Set<Subsystem> getRequirements() {
                return reqs;
            }
        };
    }

    private final Extender extender;

    public ExtenderCommands(Extender extender) {
        this.extender = extender;
    }
}
