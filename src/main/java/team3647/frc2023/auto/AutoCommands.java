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
    public final AutonomousMode blueConeCubeFlatSideMode;
    public final AutonomousMode blueConeCubeBalanceBumpSideMode;
    public final AutonomousMode blueConeBalance;
    public final AutonomousMode blueJustScore;
    public final AutonomousMode blueConeConeBalanceFlatSideMode;

    public final AutonomousMode redConeCubeFlatSideMode;
    public final AutonomousMode redConeCubeBalanceBumpSideMode;
    public final AutonomousMode redConeBalance;
    public final AutonomousMode redJustScore;
    public final AutonomousMode redConeConeBalanceFlatSideMode;

    public AutoCommands(
            SwerveDrive drive,
            SwerveDriveKinematics driveKinematics,
            Superstructure superstructure) {
        this.drive = drive;
        this.driveKinematics = driveKinematics;
        this.superstructure = superstructure;
        // blue side modes
        blueConeCubeFlatSideMode =
                new AutonomousMode(
                        coneCubeFlatSide(Alliance.Blue),
                        Trajectories.Blue.ConeCubeFlatSide.kFirstPathInitial,
                        Trajectories.Blue.ConeCubeFlatSide.kFirstPathInitial);
        blueConeConeBalanceFlatSideMode =
                new AutonomousMode(
                        coneConeBalanceFlatSide(Alliance.Blue),
                        Trajectories.Blue.ConeConeBalanceFlatSide.kFirstPathInitial,
                        Trajectories.Blue.ConeConeBalanceFlatSide.kFirstPathInitial);
        blueConeCubeBalanceBumpSideMode =
                new AutonomousMode(
                        coneCubeBalanceBumpSide(Alliance.Blue),
                        Trajectories.Blue.ConeCubeBumpSide.kFirstPathInitial,
                        Trajectories.Blue.ConeCubeBumpSide.kFirstPathInitial);
        blueConeBalance =
                new AutonomousMode(
                        justScoreBalance(
                                () -> SuperstructureState.coneThreeReversed, Alliance.Blue),
                        Trajectories.Blue.ConeBalance.kFirstPathInitial,
                        Trajectories.Blue.ConeBalance.kFirstPathInitial);
        blueJustScore =
                new AutonomousMode(
                        justScore(() -> SuperstructureState.coneThreeReversed),
                        getJustScore(FieldConstants.kBlueSix),
                        flipForPP(getJustScore(FieldConstants.kBlueSix)));

        // red side modes
        redConeCubeFlatSideMode =
                new AutonomousMode(
                        coneCubeFlatSide(Alliance.Red),
                        FieldConstants.flipBluePose(
                                Trajectories.Blue.ConeCubeFlatSide.kFirstPathInitial),
                        flipForPP(Trajectories.Blue.ConeCubeFlatSide.kFirstPathInitial));
        redConeCubeBalanceBumpSideMode =
                new AutonomousMode(
                        coneCubeBalanceBumpSide(Alliance.Red),
                        FieldConstants.flipBluePose(
                                Trajectories.Blue.ConeCubeBumpSide.kFirstPathInitial),
                        flipForPP(Trajectories.Blue.ConeCubeBumpSide.kFirstPathInitial));
        redConeConeBalanceFlatSideMode =
                new AutonomousMode(
                        coneConeBalanceFlatSide(Alliance.Red),
                        Trajectories.Blue.ConeConeBalanceFlatSide.kFirstPathInitial,
                        Trajectories.Blue.ConeConeBalanceFlatSide.kFirstPathInitial);
        redConeBalance =
                new AutonomousMode(
                        justScoreBalance(() -> SuperstructureState.coneThreeReversed, Alliance.Red),
                        getJustScore(FieldConstants.kRedFour),
                        flipForPP(getJustScore(FieldConstants.kBlueFour)));
        redJustScore =
                new AutonomousMode(
                        justScore(() -> SuperstructureState.coneThreeReversed),
                        getJustScore(FieldConstants.kRedFour),
                        flipForPP(getJustScore(FieldConstants.kRedFour)));
    }

    public static Pose2d getJustScore(Pose2d pose) {
        return new Pose2d(
                pose.getTranslation(), pose.getRotation().rotateBy(FieldConstants.kOneEighty));
    }

    public Pose2d flipForPP(Pose2d pose) {
        return new Pose2d(
                new Translation2d(pose.getX(), FieldConstants.kFieldWidth - pose.getY()),
                new Rotation2d(pose.getRotation().getCos() * -1, pose.getRotation().getSin()));
    }

    private Command getSupestructureSequenceConeCube() {
        return Commands.sequence(
                superstructure.goToStateParallel(SuperstructureState.coneThreeReversed),
                superstructure.scoreAndStow(0).withTimeout(1.2),
                Commands.waitSeconds(1 - 0.5),
                Commands.parallel(
                                superstructure.groundIntake(),
                                superstructure.rollersCommands.intake())
                        .withTimeout(2.5),
                superstructure.stow().withTimeout(2),
                superstructure.goToStateParallel(SuperstructureState.cubeThreeReversed),
                superstructure.scoreAndStowCube(1));
    }

    private Command getSupestructureSequenceThreePieces(
            Command twoPieceCommand,
            double path1time,
            double path2time,
            double path3time,
            double path4time) {
        return Commands.sequence(
                twoPieceCommand.withTimeout(path1time + path2time + 3),
                superstructure.stow(),
                Commands.waitSeconds(path3time * 0.2 - 3),
                Commands.parallel(
                                superstructure.groundIntake(),
                                superstructure.rollersCommands.intake())
                        .withTimeout(path3time * 0.8 - 4 + path4time * 0.2 + 3),
                superstructure.stow().withTimeout(0.4 + path4time * 0.4),
                superstructure.goToStateParallel(SuperstructureState.coneThree),
                superstructure.scoreAndStow(0.5));
    }

    private Command getSupestructureSequenceTwoPieceBalanceFlatSide(
            double path1time, double path2time, double path3time) {
        return Commands.sequence(
                superstructure.goToStateParallel(SuperstructureState.coneThreeReversed),
                superstructure.scoreAndStow(0).withTimeout(1.2),
                Commands.waitSeconds(path1time * 0.2 - 3),
                Commands.parallel(
                                superstructure.groundIntake(),
                                superstructure.rollersCommands.intake())
                        .withTimeout(path1time * 0.8 - 4 + path1time * 0.2 + 3),
                superstructure.stow().withTimeout(0.8),
                Commands.waitSeconds(path2time),
                superstructure.goToStateParallel(SuperstructureState.coneThree),
                superstructure.scoreAndStow(0.5));
    }

    public Command coneCubeBalanceBumpSide(Alliance color) {
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

    public Command coneCubeFlatSide(Alliance color) {
        Command drivetrainSequence =
                Commands.sequence(
                        Commands.waitSeconds(1.75), // score cone
                        followTrajectory(
                                PathPlannerTrajectory.transformTrajectoryForAlliance(
                                        Trajectories.Blue.ConeCubeFlatSide.kFirstTrajectory,
                                        color)),
                        // intake rollers don't need waiting
                        followTrajectory(
                                PathPlannerTrajectory.transformTrajectoryForAlliance(
                                        Trajectories.Blue.ConeCubeFlatSide.kSecondTrajectory,
                                        color)),
                        Commands.waitSeconds(1.2),
                        followTrajectory(
                                PathPlannerTrajectory.transformTrajectoryForAlliance(
                                        Trajectories.Blue.ConeCubeFlatSide.kThirdTrajectory,
                                        color)),
                        // intake rollers no wait
                        followTrajectory(
                                PathPlannerTrajectory.transformTrajectoryForAlliance(
                                        Trajectories.Blue.ConeCubeFlatSide.kFourthTrajectory,
                                        color)));
        return Commands.parallel(
                drivetrainSequence,
                getSupestructureSequenceThreePieces(
                        getSupestructureSequenceConeCube(),
                        Trajectories.Blue.ConeCubeFlatSide.kFirstTrajectory.getTotalTimeSeconds(),
                        Trajectories.Blue.ConeCubeFlatSide.kSecondTrajectory.getTotalTimeSeconds(),
                        Trajectories.Blue.ConeCubeFlatSide.kThirdTrajectory.getTotalTimeSeconds(),
                        Trajectories.Blue.ConeCubeFlatSide.kFourthTrajectory
                                .getTotalTimeSeconds()));
    }

    public Command coneConeBalanceFlatSide(Alliance color) {
        Command drivetrainSequence =
                Commands.sequence(
                        Commands.waitSeconds(1.75), // score cone
                        followTrajectory(
                                PathPlannerTrajectory.transformTrajectoryForAlliance(
                                        Trajectories.Blue.ConeConeBalanceFlatSide.kFirstTrajectory,
                                        color)),
                        // intake rollers don't need waiting
                        followTrajectory(
                                PathPlannerTrajectory.transformTrajectoryForAlliance(
                                        Trajectories.Blue.ConeConeBalanceFlatSide.kSecondTrajectory,
                                        color)),
                        Commands.waitSeconds(1.5),
                        followTrajectory(
                                PathPlannerTrajectory.transformTrajectoryForAlliance(
                                        Trajectories.Blue.ConeConeBalanceFlatSide.kThirdTrajectory,
                                        color)));
        return Commands.parallel(
                drivetrainSequence,
                getSupestructureSequenceTwoPieceBalanceFlatSide(
                        Trajectories.Blue.ConeConeBalanceFlatSide.kFirstTrajectory
                                .getTotalTimeSeconds(),
                        Trajectories.Blue.ConeConeBalanceFlatSide.kSecondTrajectory
                                .getTotalTimeSeconds(),
                        Trajectories.Blue.ConeConeBalanceFlatSide.kThirdTrajectory
                                .getTotalTimeSeconds()));
    }

    public Command justScore(Supplier<SuperstructureState> state) {
        return Commands.sequence(
                superstructure.goToStateParallel(state.get()).withTimeout(1),
                Commands.waitSeconds(0.5),
                superstructure.scoreStowHalfSecDelay());
    }

    public Command justScoreBalance(Supplier<SuperstructureState> state, Alliance alliance) {
        var drivetrainSequence =
                Commands.sequence(
                        Commands.waitSeconds(6),
                        followTrajectory(
                                PathPlannerTrajectory.transformTrajectoryForAlliance(
                                        Trajectories.Blue.ConeBalance.kFirstTrajectory, alliance)),
                        superstructure
                                .drivetrainCommands
                                .robotRelativeDrive(
                                        new Translation2d(), Rotation2d.fromDegrees(5), 0.3)
                                .withTimeout(0.2),
                        superstructure.drivetrainCommands.robotRelativeDrive(
                                new Translation2d(), FieldConstants.kZero, 0.3));
        return Commands.parallel(drivetrainSequence, justScore(state));
    }

    public AutonomousMode getJustScoreBlue(SuperstructureState state) {
        return new AutonomousMode(
                justScore(() -> state),
                new Pose2d(1.8, 3.26, FieldConstants.kZero),
                new Pose2d(1.8, 3.26, FieldConstants.kZero));
    }

    public AutonomousMode getJustScoreRed(SuperstructureState state) {
        return new AutonomousMode(
                justScore(() -> state),
                FieldConstants.flipBluePose(new Pose2d(1.8, 3.26, FieldConstants.kZero)),
                flipForPP(new Pose2d(1.8, 3.26, FieldConstants.kZero)));
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
