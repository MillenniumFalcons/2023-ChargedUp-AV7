package team3647.frc2023.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.Set;
import java.util.function.DoubleSupplier;
import team3647.frc2023.subsystems.Rollers;

public class RollersCommands {

    public Command intakeCone() {
        return Commands.run(rollers::intakeCone, rollers);
    }

    public Command intakeCube() {
        return Commands.run(rollers::intakeCube, rollers);
    }

    public Command intakeGround() {
        return Commands.run(rollers::intakeGround, rollers);
    }

    public Command outCone() {
        return Commands.runEnd(rollers::outtakeCone, rollers::end, rollers);
    }

    public Command outCube() {
        return Commands.runEnd(rollers::outtakeCube, rollers::end, rollers);
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
