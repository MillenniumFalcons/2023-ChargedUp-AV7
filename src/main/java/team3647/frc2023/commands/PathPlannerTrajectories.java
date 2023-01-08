package team3647.frc2023.commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import team3647.frc2023.constants.AutoConstants;

public class PathPlannerTrajectories {
    public static final PathPlannerTrajectory spinPath =
            PathPlanner.loadPath(
                    "spin",
                    AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecSq);
    public static final Pose2d spinStartPose = spinPath.getInitialPose();
}
