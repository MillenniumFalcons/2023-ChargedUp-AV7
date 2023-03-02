package team3647.frc2023.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import team3647.frc2023.robot.AllianceFlipUtil;

public class AutonomousMode {
    public AutonomousMode(Command autoCommand, Pose2d initialPose) {
        this.autoCommand = autoCommand;
        this.initialPose = initialPose;
    }

    private final Command autoCommand;

    public Command getAutoCommand() {
        return autoCommand;
    }

    private final Pose2d initialPose;

    public Pose2d getInitialPose() {
        return AllianceFlipUtil.apply(initialPose);
    }
}
