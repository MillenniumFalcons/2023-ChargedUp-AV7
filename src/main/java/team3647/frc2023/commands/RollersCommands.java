package team3647.frc2023.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

    public RollersCommands(Rollers rollers) {
        this.rollers = rollers;
    }

    private final Rollers rollers;
}
