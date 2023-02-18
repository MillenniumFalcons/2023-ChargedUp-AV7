package team3647.frc2023.commands;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import team3647.frc2023.constants.AutoConstants;
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

    public Command bottom1C1B() {
        SequentialCommandGroup drivetrainSequence = new SequentialCommandGroup(
            getCommand("S_P1"), 
            getCommand("P1_SB"), 
            getCommand("bSB_balance"));
       // ParallelCommandGroup command = new ParallelCommandGroup(drivetrain);
       
        return drivetrainSequence;
    }

    public Command top1C1B() {
        SequentialCommandGroup drivetrainSequence = new SequentialCommandGroup(
            getCommand("S_P4"), 
            getCommand("P4_SB"), 
            getCommand("tSB_balance"));
        ParallelCommandGroup command = new ParallelCommandGroup(drivetrainSequence);
        return command;
    }
     
    public SequentialCommandGroup getCommand(String pathName) {
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
            case "tSB_balance":
                trajectory = PathPlannerTrajectories.topSB_balance;
                break;
            default:
                trajectory = PathPlannerTrajectories.bottomS_P1;
        }

        return new SequentialCommandGroup(
        new InstantCommand(() -> {
          // Reset odometry for the first path you run during auto
          if(true){
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
            false,
            drive)
    );
    }
}
