package team3647.frc2023.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public class AutonomousMode {
    public AutonomousMode(Command autoCommand, Pose2d initialPose, Pose2d ppinitial) {
        this.autoCommand = autoCommand;
        this.initialPose = initialPose;
        this.ppInitial = ppinitial;
    }

    private final Command autoCommand;

    public Command getAutoCommand() {
        return autoCommand;
    }

    private final Pose2d initialPose;
    private final Pose2d ppInitial;

    public Pose2d getInitialPose() {
        return initialPose;
    }

    public Pose2d getPathplannerPose2d() {
        return ppInitial;
    }
}
