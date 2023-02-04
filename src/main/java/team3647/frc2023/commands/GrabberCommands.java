package team3647.frc2023.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import java.util.function.DoubleSupplier;
import team3647.frc2023.subsystems.Grabber;

public class GrabberCommands {

    public Command openloop(DoubleSupplier demand) {
        return Commands.run(() -> grabber.setOpenloop(demand.getAsDouble()), this.grabber);
    }

    public Command initClose() {
        return setAngle(100);
    }

    public Command setAngle(double degSetpoint) {
        return new FunctionalCommand(
                () -> {},
                () -> grabber.setAngle(degSetpoint),
                interrupted -> {},
                () -> Math.abs(grabber.getAngle() - degSetpoint) < 0.05,
                grabber);
    }

    private final Grabber grabber;

    public GrabberCommands(Grabber grabber) {
        this.grabber = grabber;
    }
}
