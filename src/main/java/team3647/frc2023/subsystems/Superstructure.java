package team3647.frc2023.subsystems;

import team3647.frc2023.commands.DrivetrainCommands;

public class Superstructure {

    public void periodic(double timestamp) {
        // Add (opposite) of tangential velocity about goal + angular velocity in local frame.

    }

    // keep this at the bottom
    public Superstructure(SwerveDrive drive) {
        this.drive = drive;

        drivetrainCommands = new DrivetrainCommands(drive);
    }

    private final SwerveDrive drive;
    public final DrivetrainCommands drivetrainCommands;
}
