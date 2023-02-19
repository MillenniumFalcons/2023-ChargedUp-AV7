package team3647.frc2023.commands;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import team3647.frc2023.constants.AutoConstants;
import team3647.frc2023.constants.SwerveDriveConstants;
import team3647.frc2023.subsystems.Superstructure;
import team3647.frc2023.subsystems.SwerveDrive;

public class AutoCommands {
    private final SwerveDrive drive;
    private final SwerveDriveKinematics driveKinematics;
    private final Superstructure superstructure;

    public AutoCommands(
            SwerveDrive drive,
            SwerveDriveKinematics driveKinematics,
            Superstructure superstructure) {
        this.drive = drive;
        this.driveKinematics = driveKinematics;
        this.superstructure = superstructure;
    }
    // arm rotate for cone
    // 2 second intake overestimate?
    // 1 second score underestimate?
    
    public Command bottom2C1B() {
        SequentialCommandGroup drivetrainSequence = new SequentialCommandGroup(
                new WaitCommand(1),
                getTraj("S_P1", true),
                new WaitCommand(2),
                getTraj("P1_SB", false),
                new WaitCommand(1),
                getTraj("SB_P2", false),
                new WaitCommand(2),
                getTraj("P2_SC", false),
                new WaitCommand(1));

        return new ParallelCommandGroup(drivetrainSequence);
    }

    //arm rotate for cone
    public Command bottom1C2B() {
        SequentialCommandGroup drivetrainSequence = new SequentialCommandGroup(
                new WaitCommand(1),
                getTraj("S_P1", true),
                new WaitCommand(2),
                getTraj("P1_SB", false),
                new WaitCommand(1),
                getTraj("SB_P2", false),
                new WaitCommand(2),
                getTraj("P2_SB", false),
                new WaitCommand(1));

        return new ParallelCommandGroup(drivetrainSequence);
    }

    public Command bottom1C1B() {
        SequentialCommandGroup drivetrainSequence = new SequentialCommandGroup(
                new WaitCommand(1),
                getTraj("S_P1", true),
                new WaitCommand(2),
                getTraj("P1_SB", false),
                new WaitCommand(1),
                getTraj("bSB_balance", false),
                superstructure.drivetrainCommands
                        .balance(
                                SwerveDriveConstants.kPitchController,
                                SwerveDriveConstants.kRollController));

        return new ParallelCommandGroup(drivetrainSequence);
    }

    public Command top1C1B() {
        SequentialCommandGroup drivetrainSequence = new SequentialCommandGroup(
                new WaitCommand(1),
                getTraj("S_P4", true),
                new WaitCommand(2),
                getTraj("P4_SB", false),
                new WaitCommand(1),
                getTraj("tSB_balance", false),
                superstructure.drivetrainCommands
                        .balance(
                                SwerveDriveConstants.kPitchController,
                                SwerveDriveConstants.kRollController));

        return new ParallelCommandGroup(drivetrainSequence);
    }

    public Command centerR1C() {
        SequentialCommandGroup drivetrainSequence = new SequentialCommandGroup(
                new WaitCommand(1),
                getTraj("RSC_balance", true),
                superstructure.drivetrainCommands
                        .balance(
                                SwerveDriveConstants.kPitchController,
                                SwerveDriveConstants.kRollController));

        return new ParallelCommandGroup(drivetrainSequence);
    }

    public Command centerL1C() {
        SequentialCommandGroup drivetrainSequence = new SequentialCommandGroup(
                new WaitCommand(1),
                getTraj("LSC_balance", true),
                superstructure.drivetrainCommands
                        .balance(
                                SwerveDriveConstants.kPitchController,
                                SwerveDriveConstants.kRollController));

        return new ParallelCommandGroup(drivetrainSequence);
    }

    public SequentialCommandGroup getTraj(String pathName, boolean firstPath) {
        final PathPlannerTrajectory trajectory;
        switch (pathName) {
            case "S_P1":
                trajectory = PathPlannerTrajectories.bottomS_P1;
                break;
            case "P1_SB":
                trajectory = PathPlannerTrajectories.bottomP1_SB;
                break;
            case "bSB_balance":
                trajectory = PathPlannerTrajectories.bottomSB_Bal;
                break;
            case "S_P4":
                trajectory = PathPlannerTrajectories.topS_P4;
                break;
            case "P4_SB":
                trajectory = PathPlannerTrajectories.topP4_SB;
                break;
            case "RSC_balance":
                trajectory = PathPlannerTrajectories.centerRSC_balance;
                break;
            case "LSC_balance":
                trajectory = PathPlannerTrajectories.centerLSC_balance;
                break;
            case "SB_P2":
                trajectory = PathPlannerTrajectories.bottomSB_P2;
                break;
            case "P2_SC":
                trajectory = PathPlannerTrajectories.bottomP2_SC;
                break;
            case "P2_SB":
                trajectory = PathPlannerTrajectories.bottomP2_SB;
                break;
            default:
                trajectory = PathPlannerTrajectories.bottomS_P1;
        }

        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    // reset odometry
                    if (firstPath) {
                        drive.setRobotPose(trajectory.getInitialHolonomicPose());
                    }
                }),
                new PPSwerveControllerCommand(
                        trajectory,
                        drive::getPose,
                        driveKinematics,
                        AutoConstants.kXController,
                        AutoConstants.kYController,
                        AutoConstants.kRotController,
                        drive::setModuleStates,
                        // false runs blue
                        false,
                        drive));
    }
}
