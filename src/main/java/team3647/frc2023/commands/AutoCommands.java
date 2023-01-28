package team3647.frc2023.commands;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import team3647.frc2023.constants.AutoConstants;
import team3647.frc2023.subsystems.SwerveDrive;

public class AutoCommands {
    private final SwerveDrive drive;

    public AutoCommands(SwerveDrive drive) {
        this.drive = drive;
    }

    public PPSwerveControllerCommand getPathCommand() {
        PathPlannerTrajectory trajectory = PathPlannerTrajectories.spinPath;
        return new PPSwerveControllerCommand(
                trajectory,
                drive::getPose,
                AutoConstants.kXController,
                AutoConstants.kYController,
                AutoConstants.kRotController,
                drive::setChasisSpeeds,
                drive);
    }
}
