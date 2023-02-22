package team3647.frc2023.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.Set;
import team3647.frc2023.subsystems.Grabber;

public class GrabberCommands {

    public Command closeGrabber() {
        return Commands.run(() -> grabber.close(), this.grabber)
                .finallyDo(interrupted -> Commands.run(() -> {}, grabber));
    }

    public Command closeAndRollIn() {
        return Commands.run(() -> grabber.closeAndRoll(), this.grabber)
                .finallyDo(interrupted -> Commands.run(() -> {}, grabber));
    }

    public Command openGrabber() {
        return Commands.run(() -> grabber.open(), this.grabber)
                .finallyDo(interrupted -> Commands.run(() -> {}, grabber));
    }

    public Command intakeCone() {
        return Commands.run(() -> grabber.intakeCone(), this.grabber)
                .finallyDo(interrupted -> Commands.run(() -> {}, grabber));
    }

    public Command intakeCube() {
        return Commands.run(() -> grabber.intakeCube(), this.grabber)
                .finallyDo(interrupted -> Commands.run(() -> {}, grabber));
    }

    public Command stop() {
        return Commands.run(() -> grabber.setOpenloop(0), this.grabber)
                .finallyDo(interrupted -> Commands.run(() -> {}, grabber));
    }

    public Command holdPositionAtCall() {
        return new Command() {
            double pos = 0;

            @Override
            public void initialize() {
                pos = grabber.getPosition() + 2000;
            }

            @Override
            public void execute() {
                grabber.setPositionNative(pos);
            }

            @Override
            public Set<Subsystem> getRequirements() {
                return Set.of(grabber);
            }
        };
    }

    // public Command holdPosition() {
    //     return Commands.run(() -> grabber.holdCurrentPosition(), this.grabber)
    //             .finallyDo(interrupted -> Commands.run(() -> {}, grabber));
    // }

    private final Grabber grabber;

    public GrabberCommands(Grabber grabber) {
        this.grabber = grabber;
    }
}
