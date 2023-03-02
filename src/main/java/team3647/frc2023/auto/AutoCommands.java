package team3647.frc2023.auto;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import java.util.function.Supplier;
import team3647.frc2023.constants.AutoConstants;
import team3647.frc2023.subsystems.Superstructure;
import team3647.frc2023.subsystems.SwerveDrive;
import team3647.frc2023.util.SuperstructureState;

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

    private Command getSupestructureSequenceConeCube() {
        return Commands.sequence(
                superstructure.goToStateParallel(SuperstructureState.coneThreeReversed),
                superstructure.scoreAndStow(0.5).withTimeout(1.25),
                new WaitCommand(1),
                Commands.parallel(
                                superstructure.groundIntake(),
                                superstructure.rollersCommands.intake())
                        .withTimeout(2),
                superstructure.stow(),
                new WaitCommand(3),
                superstructure.goToStateParallel(SuperstructureState.cubeThreeReversed),
                superstructure.scoreAndStow(0.5));
    }

    private Command getSupestructureSequenceThreePieces(
            Command twoPieceCommand,
            double path1time,
            double path2time,
            double path3time,
            double path4time) {
        return Commands.sequence(
                twoPieceCommand.withTimeout(path1time + path2time),
                Commands.waitSeconds(1),
                Commands.parallel(
                                superstructure.groundIntake(),
                                superstructure.rollersCommands.intake())
                        .withTimeout(path3time - 1 + path4time * 0.2),
                superstructure.stow(),
                Commands.waitSeconds(path4time * 0.6),
                superstructure.goToStateParallel(SuperstructureState.cubeThreeReversed),
                superstructure.scoreAndStow(0.5));
    }

    public Command coneCubeClimbBumpSide() {
        Command drivetrainSequence =
                Commands.sequence(
                        Commands.waitSeconds(1.75), // score cone
                        followTrajectoryAutoColor(
                                Trajectories.Blue.ConeCubeBumpSide.kFirstTrajectory),
                        // rollers don't need waiting
                        followTrajectoryAutoColor(
                                Trajectories.Blue.ConeCubeBumpSide.kSecondTrajectory),
                        Commands.waitSeconds(1),
                        followTrajectoryAutoColor(Trajectories.Blue.ConeCubeBumpSide.kGoToBalance));
        return Commands.parallel(drivetrainSequence, getSupestructureSequenceConeCube());
    }

    public Command coneCubeConeFlatSide() {
        Command drivetrainSequence =
                Commands.sequence(
                        Commands.waitSeconds(1.75), // score cone
                        followTrajectoryAutoColor(
                                Trajectories.Blue.ConeCubeConeFlat.kFirstTrajectory),
                        // rollers don't need waiting
                        followTrajectoryAutoColor(
                                Trajectories.Blue.ConeCubeConeFlat.kSecondTrajectory),
                        Commands.waitSeconds(1),
                        followTrajectoryAutoColor(
                                Trajectories.Blue.ConeCubeConeFlat.kThirdTrajectory),
                        followTrajectoryAutoColor(
                                Trajectories.Blue.ConeCubeConeFlat.kFourthTrajectory));
        return Commands.parallel(
                drivetrainSequence,
                getSupestructureSequenceThreePieces(
                        getSupestructureSequenceConeCube(),
                        Trajectories.Blue.ConeCubeConeFlat.kFirstTrajectory.getTotalTimeSeconds(),
                        Trajectories.Blue.ConeCubeConeFlat.kSecondTrajectory.getTotalTimeSeconds(),
                        Trajectories.Blue.ConeCubeConeFlat.kThirdTrajectory.getTotalTimeSeconds(),
                        Trajectories.Blue.ConeCubeConeFlat.kFourthTrajectory
                                .getTotalTimeSeconds()));
    }

    public Command justScore(Supplier<SuperstructureState> state) {
        return Commands.sequence(
                superstructure.goToStateParallel(state.get()),
                Commands.waitSeconds(0.5),
                superstructure.scoreStowHalfSecDelay());
    }

    public final AutonomousMode coneCubeConeFlatSideMode =
            new AutonomousMode(
                    coneCubeConeFlatSide(),
                    Trajectories.Blue.ConeCubeConeFlat.kFirstTrajectory.getInitialHolonomicPose());
    public final AutonomousMode coneCubeClimbBumpSideMode =
            new AutonomousMode(
                    coneCubeClimbBumpSide(),
                    Trajectories.Blue.ConeCubeBumpSide.kFirstTrajectory.getInitialHolonomicPose());

    public AutonomousMode getJustScore(SuperstructureState state) {
        return new AutonomousMode(
                justScore(() -> state), new Pose2d(1.8, 3.26, Rotation2d.fromDegrees(0)));
    }

    public Command followTrajectoryAutoColor(PathPlannerTrajectory trajectory) {
        return followTrajectory(trajectory, true);
    }

    public Command followTrajectory(PathPlannerTrajectory trajectory, boolean automaticAlliance) {
        return new PPSwerveControllerCommand(
                trajectory,
                drive::getOdoPose,
                driveKinematics,
                AutoConstants.kXController,
                AutoConstants.kYController,
                AutoConstants.kRotController,
                drive::setModuleStates,
                // false runs blue
                automaticAlliance,
                drive);
    }
}
