package team3647.frc2023.subsystems;

import team3647.frc2023.commands.DrivetrainCommands;
import team3647.frc2023.commands.PivotCommands;

public class Superstructure {

    public void periodic(double timestamp) {
        // Add (opposite) of tangential velocity about goal + angular velocity in local frame.

    }

    // keep this at the bottom
    public Superstructure(SwerveDrive drive, Pivot pivot) {
        this.drive = drive;
        this.pivot = pivot;

        drivetrainCommands = new DrivetrainCommands(drive);
        pivotCommands = new PivotCommands(pivot);
    }

    private final SwerveDrive drive;
    private final Pivot pivot;
    public final DrivetrainCommands drivetrainCommands;
    public final PivotCommands pivotCommands;
}
