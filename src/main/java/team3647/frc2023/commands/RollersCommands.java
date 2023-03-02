package team3647.frc2023.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.Set;
import java.util.function.DoubleSupplier;
import team3647.frc2023.subsystems.Rollers;

public class RollersCommands {

    public Command intake() {
        return Commands.runEnd(rollers::intake, rollers::end, rollers);
    }

    public Command out() {
        return Commands.runEnd(rollers::outtake, rollers::end, rollers);
    }

    public Command openloop(DoubleSupplier speed) {
        return Commands.runEnd(
                () -> rollers.setOpenloop(speed.getAsDouble()), rollers::end, rollers);
    }

    private final Set<Subsystem> requirements;

    public RollersCommands(Rollers rollers) {
        this.rollers = rollers;
        this.requirements = Set.of(rollers);
    }

    private final Rollers rollers;
}
