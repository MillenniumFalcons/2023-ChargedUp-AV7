package team3647.frc2023.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.Command;
import team3647.frc2023.constants.AutoConstants;
import team3647.frc2023.constants.SwerveDriveConstants;
import team3647.frc2023.subsystems.SwerveDrive;

public class AutoCommands {
    private final SwerveDrive drive;

    public AutoCommands(SwerveDrive drive) {
        this.drive = drive;
    }

    public Command follow(PathPlannerTrajectory trajectory) {
        return new PPSwerveControllerCommand(
                trajectory,
                drive::getPose,
                AutoConstants.kXController,
                AutoConstants.kYController,
                AutoConstants.kRotController,
                drive::setChasisSpeeds,
                drive);
    }

    public Command goToPoint(PathPoint point, PathConstraints constraints) {
        return follow(
                PathPlanner.generatePath(
                        constraints,
                        PathPoint.fromCurrentHolonomicState(
                                drive.getPose(),
                                SwerveDriveConstants.kDriveKinematics.toChassisSpeeds(
                                        drive.getModuleStates())),
                        point));
    }
}
