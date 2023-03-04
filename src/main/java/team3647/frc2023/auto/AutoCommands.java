package team3647.frc2023.auto;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.Supplier;
import team3647.frc2023.constants.AutoConstants;
import team3647.frc2023.constants.FieldConstants;
import team3647.frc2023.subsystems.Superstructure;
import team3647.frc2023.subsystems.SwerveDrive;
import team3647.frc2023.util.SuperstructureState;

public class AutoCommands {
    private final SwerveDrive drive;
    private final SwerveDriveKinematics driveKinematics;
    private final Superstructure superstructure;
    public final AutonomousMode blueConeCubeConeFlatSideMode;
    public final AutonomousMode blueConeCubeClimbBumpSideMode;
    public final AutonomousMode redConeCubeConeFlatSideMode;
    public final AutonomousMode redConeCubeClimbBumpSideMode;

    public AutoCommands(
            SwerveDrive drive,
            SwerveDriveKinematics driveKinematics,
            Superstructure superstructure) {
        this.drive = drive;
        this.driveKinematics = driveKinematics;
        this.superstructure = superstructure;
        blueConeCubeConeFlatSideMode =
                new AutonomousMode(
                        coneCubeConeFlatSide(Alliance.Blue),
                        Trajectories.Blue.ConeCubeConeFlat.kFirstPathInitial,
                        Trajectories.Blue.ConeCubeConeFlat.kFirstPathInitial);
        blueConeCubeClimbBumpSideMode =
                new AutonomousMode(
                        coneCubeClimbBumpSide(Alliance.Blue),
                        Trajectories.Blue.ConeCubeBumpSide.kFirstPathInitial,
                        Trajectories.Blue.ConeCubeBumpSide.kFirstPathInitial);
        redConeCubeConeFlatSideMode =
                new AutonomousMode(
                        coneCubeConeFlatSide(Alliance.Red),
                        FieldConstants.flipBluePose(
                                Trajectories.Blue.ConeCubeBumpSide.kFirstPathInitial),
                        flipForPP(Trajectories.Blue.ConeCubeBumpSide.kFirstPathInitial));
        redConeCubeClimbBumpSideMode =
                new AutonomousMode(
                        coneCubeClimbBumpSide(Alliance.Red),
                        FieldConstants.flipBluePose(
                                Trajectories.Blue.ConeCubeBumpSide.kFirstPathInitial),
                        flipForPP(Trajectories.Blue.ConeCubeBumpSide.kFirstPathInitial));
    }

    public static Pose2d getJustScore(Pose2d pose) {
        return new Pose2d(
                pose.getTranslation(), pose.getRotation().rotateBy(Rotation2d.fromDegrees(180)));
    }

    private Command getSupestructureSequenceConeCube() {
        return Commands.sequence(
                superstructure.goToStateParallel(SuperstructureState.coneThreeReversed),
                superstructure.scoreAndStow(0).withTimeout(1.2),
                Commands.waitSeconds(1),
                Commands.parallel(
                                superstructure.groundIntake(),
                                superstructure.rollersCommands.intake())
                        .withTimeout(1),
                superstructure.stow().withTimeout(3),
                superstructure.goToStateParallel(SuperstructureState.cubeThreeReversed),
                superstructure.scoreAndStow(1));
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

    public Command coneCubeClimbBumpSide(Alliance color) {
        Command drivetrainSequence =
                Commands.sequence(
                        Commands.waitSeconds(2), // score cone
                        followTrajectory(
                                PathPlannerTrajectory.transformTrajectoryForAlliance(
                                        Trajectories.Blue.ConeCubeBumpSide.kFirstTrajectory,
                                        color)),
                        // rollers don't need waiting
                        followTrajectory(
                                PathPlannerTrajectory.transformTrajectoryForAlliance(
                                        Trajectories.Blue.ConeCubeBumpSide.kSecondTrajectory,
                                        color)),
                        Commands.waitSeconds(1),
                        followTrajectory(
                                PathPlannerTrajectory.transformTrajectoryForAlliance(
                                        Trajectories.Blue.ConeCubeBumpSide.kGoToBalance, color)));
        return Commands.parallel(drivetrainSequence, getSupestructureSequenceConeCube());
    }

    public Pose2d flipForPP(Pose2d pose) {
        return new Pose2d(
                new Translation2d(pose.getX(), FieldConstants.kFieldWidth - pose.getY()),
                new Rotation2d(pose.getRotation().getCos() * -1, pose.getRotation().getSin()));
    }

    public Command coneCubeConeFlatSide(Alliance color) {
        Command drivetrainSequence =
                Commands.sequence(
                        Commands.waitSeconds(1.75), // score cone
                        followTrajectory(
                                PathPlannerTrajectory.transformTrajectoryForAlliance(
                                        Trajectories.Blue.ConeCubeConeFlat.kFirstTrajectory,
                                        color)),
                        // rollers don't need waiting
                        followTrajectory(
                                PathPlannerTrajectory.transformTrajectoryForAlliance(
                                        Trajectories.Blue.ConeCubeConeFlat.kSecondTrajectory,
                                        color)),
                        Commands.waitSeconds(1),
                        followTrajectory(
                                PathPlannerTrajectory.transformTrajectoryForAlliance(
                                        Trajectories.Blue.ConeCubeConeFlat.kThirdTrajectory,
                                        color)),
                        followTrajectory(
                                PathPlannerTrajectory.transformTrajectoryForAlliance(
                                        Trajectories.Blue.ConeCubeConeFlat.kFourthTrajectory,
                                        color)));

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
                superstructure.goToStateParallel(state.get()).withTimeout(1),
                Commands.waitSeconds(0.5),
                superstructure.scoreStowHalfSecDelay());
    }

    public AutonomousMode getJustScoreBlue(SuperstructureState state) {
        return new AutonomousMode(
                justScore(() -> state),
                new Pose2d(1.8, 3.26, Rotation2d.fromDegrees(0)),
                new Pose2d(1.8, 3.26, Rotation2d.fromDegrees(0)));
    }

    public AutonomousMode getJustScoreRed(SuperstructureState state) {
        return new AutonomousMode(
                justScore(() -> state),
                FieldConstants.flipBluePose(new Pose2d(1.8, 3.26, Rotation2d.fromDegrees(0))),
                flipForPP(new Pose2d(1.8, 3.26, Rotation2d.fromDegrees(0))));
    }

    public Command followTrajectory(PathPlannerTrajectory trajectory) {
        return new PPSwerveControllerCommand(
                trajectory,
                drive::getPPPose,
                driveKinematics,
                AutoConstants.kXController,
                AutoConstants.kYController,
                AutoConstants.kRotController,
                drive::setModuleStates,
                // false runs blue
                false,
                drive);
    }
}
