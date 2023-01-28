package team3647.frc2023.subsystems;

public class Superstructure {
    private final SwerveDrive drive;

    public void periodic(double timestamp) {
        // Add (opposite) of tangential velocity about goal + angular velocity in local frame.

    }

    // keep this at the bottom
    public Superstructure(SwerveDrive drive) {
        this.drive = drive;
    }
}
