package team3647.frc2023.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import java.util.function.DoubleSupplier;
import team3647.frc2023.commands.DrivetrainCommands;
import team3647.frc2023.commands.ExtenderCommands;
import team3647.frc2023.commands.GrabberCommands;
import team3647.frc2023.commands.PivotCommands;

public class Superstructure {

    public void periodic(double timestamp) {
        // Add (opposite) of tangential velocity about goal + angular velocity in local frame.

    }

    // keep this at the bottom
    public Superstructure(SwerveDrive drive, Pivot pivot, Extender extender, Grabber grabber) {
        this.drive = drive;
        this.pivot = pivot;
        this.extender = extender;
        this.grabber = grabber;

        drivetrainCommands = new DrivetrainCommands(drive);
        pivotCommands = new PivotCommands(pivot);
        extenderCommands = new ExtenderCommands(extender);
        grabberCommands = new GrabberCommands(grabber);
    }

    public Command setPivotAngle(DoubleSupplier setpoint) {
        return new FunctionalCommand(
                () -> {},
                () -> pivot.setAngle(setpoint.getAsDouble(), extender::getLengthMeters),
                interrupted -> {},
                () -> Math.abs(pivot.getAngle() - setpoint.getAsDouble()) < 0.05,
                pivot);
    }

    private final SwerveDrive drive;
    private final Pivot pivot;
    private final Extender extender;
    private final Grabber grabber;
    public final DrivetrainCommands drivetrainCommands;
    public final PivotCommands pivotCommands;
    public final ExtenderCommands extenderCommands;
    public final GrabberCommands grabberCommands;
}
