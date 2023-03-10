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
    public final AutonomousMode blueConeBalance;
    public final AutonomousMode redConeCubeConeFlatSideMode;
    public final AutonomousMode redConeCubeClimbBumpSideMode;
    public final AutonomousMode redConeBalance;
    public final AutonomousMode redJustScore;

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
                        Trajectories.Blue.ConeCubeConeFlatSide.kFirstPathInitial,
                        Trajectories.Blue.ConeCubeConeFlatSide.kFirstPathInitial);
        blueConeCubeClimbBumpSideMode =
                new AutonomousMode(
                        coneCubeClimbBumpSide(Alliance.Blue),
                        Trajectories.Blue.ConeCubeBumpSide.kFirstPathInitial,
                        Trajectories.Blue.ConeCubeBumpSide.kFirstPathInitial);
        blueConeBalance =
                new AutonomousMode(
                        justScoreBalance(
                                () -> SuperstructureState.reverseArm(SuperstructureState.coneThree),
                                Alliance.Blue),
                        Trajectories.Blue.ConeBalance.kFirstPathInitial,
                        Trajectories.Blue.ConeBalance.kFirstPathInitial);
        redConeCubeConeFlatSideMode =
                new AutonomousMode(
                        coneCubeConeFlatSide(Alliance.Red),
                        FieldConstants.flipBluePose(
                                Trajectories.Blue.ConeCubeConeFlatSide.kFirstPathInitial),
                        flipForPP(Trajectories.Blue.ConeCubeConeFlatSide.kFirstPathInitial));
        redConeCubeClimbBumpSideMode =
                new AutonomousMode(
                        coneCubeClimbBumpSide(Alliance.Red),
                        FieldConstants.flipBluePose(
                                Trajectories.Blue.ConeCubeBumpSide.kFirstPathInitial),
                        flipForPP(Trajectories.Blue.ConeCubeBumpSide.kFirstPathInitial));
        redConeBalance =
                new AutonomousMode(
                        justScoreBalance(
                                () -> SuperstructureState.reverseArm(SuperstructureState.coneThree),
                                Alliance.Red),
                        getJustScore(FieldConstants.kRedFour),
                        flipForPP(getJustScore(FieldConstants.kBlueFour)));
        redJustScore =
                new AutonomousMode(
                        justScore(
                                () ->
                                        SuperstructureState.reverseArm(
                                                SuperstructureState.coneThree)),
                        getJustScore(FieldConstants.kRedFour),
                        flipForPP(getJustScore(FieldConstants.kRedFour)));
    }

    public static Pose2d getJustScore(Pose2d pose) {
        return new Pose2d(
                pose.getTranslation(), pose.getRotation().rotateBy(FieldConstants.kOneEighty));
    }

    private Command getSupestructureSequenceConeCube() {
        return Commands.sequence(
                superstructure.goToStateParallel(
                        () -> SuperstructureState.reverseArm(SuperstructureState.coneThree)),
                superstructure.scoreAndStow(0).withTimeout(1.2),
                Commands.waitSeconds(1 - 0.5),
                Commands.parallel(
                                superstructure.groundIntake(),
                                superstructure.rollersCommands.intake())
                        .withTimeout(2.5),
                superstructure.stow().withTimeout(2),
                superstructure.goToStateParallel(
                        () -> SuperstructureState.reverseArm(SuperstructureState.coneThree)),
                superstructure.scoreAndStowCube(1));
    }

    private Command getSupestructureSequenceThreePieces(
            Command twoPieceCommand,
            double path1time,
            double path2time,
            double path3time,
            double path4time) {
        return Commands.sequence(
                twoPieceCommand.withTimeout(path1time + path2time + 3), superstructure.stow());
        // Commands.waitSeconds(path3time * 0.2 - 3),
        // Commands.parallel(
        //                 superstructure.groundIntake(),
        //                 superstructure.rollersCommands.intake())
        //         .withTimeout(path3time * 0.8 - 4 + path4time * 0.2 + 3),
        // superstructure.stow().withTimeout(0.4 + path4time * 0.4),
        // superstructure.goToStateParallel(SuperstructureState.coneThree),
        // superstructure.scoreAndStow(0.5));
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
                                        Trajectories.Blue.ConeCubeConeFlatSide.kFirstTrajectory,
                                        color)),
                        // rollers don't need waiting
                        followTrajectory(
                                PathPlannerTrajectory.transformTrajectoryForAlliance(
                                        Trajectories.Blue.ConeCubeConeFlatSide.kSecondTrajectory,
                                        color)));
        // Commands.waitSeconds(2.5),
        // followTrajectory(
        //         PathPlannerTrajectory.transformTrajectoryForAlliance(
        //                 Trajectories.Blue.ConeCubeConeFlatSide.kThirdTrajectory,
        //                 color)),
        // followTrajectory(
        //         PathPlannerTrajectory.transformTrajectoryForAlliance(
        //                 Trajectories.Blue.ConeCubeConeFlatSide.kFourthTrajectory,
        //                 color)));

        return Commands.parallel(
                drivetrainSequence,
                getSupestructureSequenceThreePieces(
                        getSupestructureSequenceConeCube(),
                        Trajectories.Blue.ConeCubeConeFlatSide.kFirstTrajectory
                                .getTotalTimeSeconds(),
                        Trajectories.Blue.ConeCubeConeFlatSide.kSecondTrajectory
                                .getTotalTimeSeconds(),
                        Trajectories.Blue.ConeCubeConeFlatSide.kThirdTrajectory
                                .getTotalTimeSeconds(),
                        Trajectories.Blue.ConeCubeConeFlatSide.kFourthTrajectory
                                .getTotalTimeSeconds()));
    }

    public Command justScore(Supplier<SuperstructureState> state) {
        return Commands.sequence(
                superstructure.goToStateParallel(() -> state.get()).withTimeout(1),
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
