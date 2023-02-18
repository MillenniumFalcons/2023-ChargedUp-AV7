package team3647.frc2023.commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import team3647.frc2023.constants.AutoConstants;

public class PathPlannerTrajectories {
        
    public static final PathPlannerTrajectory bottomS_P1 =
            PathPlanner.loadPath(
                    "bottom S-P1",
                    AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecSq);
    public static final PathPlannerTrajectory bottomP1_SB =
            PathPlanner.loadPath(
                    "bottom P1-SB",
                    AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecSq);
    public static final PathPlannerTrajectory bottomSB_Bal =
            PathPlanner.loadPath(
                    "bottom SB-Balance",
                    AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecSq);

    public static final PathPlannerTrajectory topS_P4 =
            PathPlanner.loadPath(
                    "top S-P4",
                    AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecSq);

    public static final PathPlannerTrajectory topP4_SB =
            PathPlanner.loadPath(
                    "top P4-SB",
                    AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecSq);

    public static final PathPlannerTrajectory topSB_balance =
            PathPlanner.loadPath(
                    "top SB-balance",
                    AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecSq);

    public static final PathPlannerTrajectory centerRSC_balance =
            PathPlanner.loadPath(
                    "center right S-balance",
                    AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecSq);
 
    public static final PathPlannerTrajectory centerLSC_balance =
            PathPlanner.loadPath(
                    "center left S-balance",
                    AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecSq);

    public static final PathPlannerTrajectory bottomSB_P2 =
            PathPlanner.loadPath(
                    "bottom SB-P2",
                    AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecSq);

    public static final PathPlannerTrajectory bottomP2_SC =
            PathPlanner.loadPath(
                    "bottom P2-SC",
                    AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecSq);

    public static final PathPlannerTrajectory bottomP2_SB =
            PathPlanner.loadPath(
                    "bottom SB-P2",
                    AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecSq, true);
    
    public static final Pose2d spinStartPose = bottomS_P1.getInitialPose();
}
