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
import team3647.frc2023.constants.AutoConstants;
import team3647.frc2023.constants.FieldConstants;
import team3647.frc2023.subsystems.Superstructure;
import team3647.frc2023.subsystems.SwerveDrive;
import team3647.frc2023.util.SuperstructureState;

public class AutoCommands {
    private final SwerveDrive drive;
    private final SwerveDriveKinematics driveKinematics;
    private final Superstructure superstructure;

    public final AutonomousMode Test;

    public final AutonomousMode blueConeCubeCubeFlatSideMode;
    public final AutonomousMode blueConeCubeBalanceFlatSideMode;
    public final AutonomousMode blueConeCubeBalanceBumpSideMode;
    public final AutonomousMode blueConeBalance;
    public final AutonomousMode blueJustScore;

    public final AutonomousMode redConeCubeCubeFlatSideMode;
    public final AutonomousMode redConeCubeBalanceFlatSideMode;
    public final AutonomousMode redConeCubeBalanceBumpSideMode;
    public final AutonomousMode redConeBalance;
    public final AutonomousMode redJustScore;

    public AutoCommands(
            SwerveDrive drive,
            SwerveDriveKinematics driveKinematics,
            Superstructure superstructure) {
        this.drive = drive;
        this.driveKinematics = driveKinematics;
        this.superstructure = superstructure;
        Test = new AutonomousMode(drive(), Trajectories.Blue.Test.kStart);
        // blue side modes
        blueConeCubeCubeFlatSideMode =
                new AutonomousMode(
                        coneCubeCubeFlatSide(Alliance.Blue),
                        Trajectories.Blue.ConeCubeCubeFlatSide.kFirstPathInitial);
        blueConeCubeBalanceFlatSideMode =
                new AutonomousMode(
                        coneCubeBalanceFlatSide(Alliance.Blue),
                        Trajectories.Blue.ConeCubeBalanceFlatSide.kFirstPathInitial);
        blueConeCubeBalanceBumpSideMode =
                new AutonomousMode(
                        coneCubeBalanceBumpSide(Alliance.Blue),
                        Trajectories.Blue.ConeCubeBalanceBumpSide.kFirstPathInitial);
        blueConeBalance =
                new AutonomousMode(
                        justScoreBalance(SuperstructureState.coneThreeReversed, Alliance.Blue),
                        Trajectories.Blue.ConeBalance.kFirstPathInitial);
        blueJustScore =
                new AutonomousMode(
                        justScore(SuperstructureState.coneThreeReversed),
                        new Pose2d(0, 0, FieldConstants.kZero));

        // red side modes
        redConeCubeCubeFlatSideMode =
                new AutonomousMode(
                        coneCubeCubeFlatSide(Alliance.Red),
                        flipForPP(Trajectories.Blue.ConeCubeCubeFlatSide.kFirstPathInitial));
        redConeCubeBalanceFlatSideMode =
                new AutonomousMode(
                        coneCubeBalanceFlatSide(Alliance.Red),
                        flipForPP(Trajectories.Blue.ConeCubeBalanceFlatSide.kFirstPathInitial));
        redConeCubeBalanceBumpSideMode =
                new AutonomousMode(
                        coneCubeBalanceBumpSide(Alliance.Red),
                        flipForPP(Trajectories.Blue.ConeCubeBalanceBumpSide.kFirstPathInitial));

        redConeBalance =
                new AutonomousMode(
                        justScoreBalance(SuperstructureState.coneThreeReversed, Alliance.Red),
                        flipForPP(Trajectories.Blue.ConeBalance.kFirstPathInitial));
        redJustScore =
                new AutonomousMode(
                        justScore(SuperstructureState.coneThreeReversed),
                        new Pose2d(0, 0, FieldConstants.kZero));
    }

    public static Pose2d getJustScore(Pose2d pose) {
        return new Pose2d(
                pose.getTranslation(), pose.getRotation().rotateBy(FieldConstants.kOneEighty));
    }

    public Pose2d flipForPP(Pose2d pose) {
        return new Pose2d(
                new Translation2d(pose.getX(), FieldConstants.kFieldWidth - pose.getY()),
                pose.getRotation());
    }

    private Command endRightAfterExtenderRetracted() {
        return Commands.sequence(
                Commands.waitUntil(() -> superstructure.extender.getPosition() > 6000),
                Commands.waitUntil(() -> superstructure.extender.getPosition() < 6000));
    }

    private Command getSupestructureSequenceConeCubeCubeFlat(
            double firstPathTime,
            double secondPathTime,
            double thirdPathTime,
            SuperstructureState nextState) {
        return Commands.sequence(
                getSupestructureSequenceConeCubeFlat(
                        firstPathTime,
                        secondPathTime,
                        thirdPathTime,
                        SuperstructureState.cubeOneReversedLong),
                superstructure
                        .scoreAndStowCube(0.2, -1, SuperstructureState.groundIntakeCube)
                        .raceWith(endRightAfterExtenderRetracted()),
                Commands.deadline(
                                superstructure.waitForCurrentSpike(8),
                                superstructure.goToStateParallel(
                                        SuperstructureState.groundIntakeCube),
                                superstructure.rollersCommands.openloop(() -> 0.6))
                        .withTimeout(4));
    }

    private Command getSupestructureSequenceConeCubeFlat(
            double firstPathTime,
            double secondPathTime,
            double thirdPathTime,
            SuperstructureState nextState) {
        ;
        return Commands.sequence(
                justScore(
                                SuperstructureState.coneThreeReversed,
                                SuperstructureState.stowAfterConeThreeReversed)
                        .raceWith(endRightAfterExtenderRetracted()),
                Commands.deadline(
                                superstructure.waitForCurrentSpike(12),
                                superstructure.goToStateParallel(
                                        SuperstructureState.groundIntakeCube),
                                superstructure.rollersCommands.openloop(() -> 0.8))
                        .withTimeout(3),
                superstructure.stow().withTimeout(0.5),
                superstructure.goToStateParallel(SuperstructureState.cubeThreeReversed),
                superstructure
                        .scoreAndStowCube(0.2, -0.6, SuperstructureState.groundIntakeCube)
                        .raceWith(endRightAfterExtenderRetracted()),
                Commands.deadline(
                                superstructure.waitForCurrentSpike(8),
                                superstructure.goToStateParallel(
                                        SuperstructureState.groundIntakeCube),
                                superstructure.rollersCommands.openloop(() -> 0.6))
                        .withTimeout(4),
                superstructure.goToStateParallel(nextState));
    }

    private Command getSupestructureSequenceConeCubeBump() {
        return Commands.sequence(
                justScore(
                                SuperstructureState.coneThreeReversed,
                                SuperstructureState.stowAfterConeThreeReversed)
                        .raceWith(endRightAfterExtenderRetracted()),
                Commands.waitSeconds(0.5),
                Commands.deadline(
                                superstructure.waitForCurrentSpike(7),
                                superstructure.goToStateParallel(
                                        SuperstructureState.groundIntakeCube),
                                superstructure.rollersCommands.openloop(() -> 0.45))
                        .withTimeout(3),
                superstructure.stow().withTimeout(2),
                superstructure.goToStateParallel(SuperstructureState.cubeThreeReversed),
                Commands.waitSeconds(0.8),
                superstructure.scoreAndStowCube(),
                Commands.waitSeconds(2),
                Commands.deadline(
                                superstructure.waitForCurrentSpike(7),
                                superstructure.goToStateParallel(
                                        SuperstructureState.groundIntakeCube),
                                superstructure.rollersCommands.openloop(() -> 0.4))
                        .withTimeout(3));
    }

    public Command drive() {
        return followTrajectory(Trajectories.Blue.Test.kTrajectory);
    }

    public Command coneCubeBalanceBumpSide(Alliance color) {
        Command drivetrainSequence =
                Commands.sequence(
                        Commands.waitSeconds(2), // score cone
                        followTrajectory(
                                PathPlannerTrajectory.transformTrajectoryForAlliance(
                                        Trajectories.Blue.ConeCubeBalanceBumpSide.kFirstTrajectory,
                                        color)),
                        // rollers don't need waiting
                        followTrajectory(
                                PathPlannerTrajectory.transformTrajectoryForAlliance(
                                        Trajectories.Blue.ConeCubeBalanceBumpSide.kSecondTrajectory,
                                        color)),
                        Commands.waitSeconds(1),
                        followTrajectory(
                                PathPlannerTrajectory.transformTrajectoryForAlliance(
                                        Trajectories.Blue.ConeCubeBalanceBumpSide.kGoToOutside,
                                        color)),
                        followTrajectory(
                                PathPlannerTrajectory.transformTrajectoryForAlliance(
                                        Trajectories.Blue.ConeCubeBalanceBumpSide.kIntakeCube,
                                        color)));
        // ,
        return Commands.parallel(drivetrainSequence, getSupestructureSequenceConeCubeBump());
    }

    public Command coneCubeBalanceFlatSide(Alliance color) {
        Command drivetrainSequence =
                Commands.sequence(
                        Commands.waitSeconds(2), // score cone
                        followTrajectory(
                                PathPlannerTrajectory.transformTrajectoryForAlliance(
                                        Trajectories.Blue.ConeCubeBalanceFlatSide.kFirstTrajectory,
                                        color)),
                        // intake rollers don't need waiting
                        followTrajectory(
                                PathPlannerTrajectory.transformTrajectoryForAlliance(
                                        Trajectories.Blue.ConeCubeBalanceFlatSide.kSecondTrajectory,
                                        color)),
                        followTrajectory(
                                PathPlannerTrajectory.transformTrajectoryForAlliance(
                                        Trajectories.Blue.ConeCubeBalanceFlatSide.kGoToBalance,
                                        color)),
                        Commands.run(
                                        () ->
                                                drive.drive(
                                                        new Translation2d(-0.9, 0), 0, false, true),
                                        drive)
                                .until(() -> Math.abs(drive.getPitch()) < 11)
                                .withTimeout(5),

                        // lock wheels so no slip
                        superstructure.drivetrainCommands.robotRelativeDrive(
                                new Translation2d(), FieldConstants.kZero, 0.3));
        // , getSupestructureSequenceConeCubeFlat()
        return Commands.parallel(
                drivetrainSequence,
                getSupestructureSequenceConeCubeFlat(
                        Trajectories.Blue.ConeCubeBalanceFlatSide.kFirstTrajectory
                                .getTotalTimeSeconds(),
                        Trajectories.Blue.ConeCubeBalanceFlatSide.kSecondTrajectory
                                .getTotalTimeSeconds(),
                        Trajectories.Blue.ConeCubeBalanceFlatSide.kGoToBalance
                                .getTotalTimeSeconds(),
                        SuperstructureState.stowAll));
    }

    public Command coneCubeCubeFlatSide(Alliance color) {
        Command drivetrainSequence =
                Commands.sequence(
                        Commands.waitSeconds(2), // score cone
                        followTrajectory(
                                PathPlannerTrajectory.transformTrajectoryForAlliance(
                                        Trajectories.Blue.ConeCubeCubeFlatSide.kFirstTrajectory,
                                        color)),
                        // intake rollers don't need waiting
                        followTrajectory(
                                PathPlannerTrajectory.transformTrajectoryForAlliance(
                                        Trajectories.Blue.ConeCubeCubeFlatSide.kSecondTrajectory,
                                        color)),
                        followTrajectory(
                                PathPlannerTrajectory.transformTrajectoryForAlliance(
                                        Trajectories.Blue.ConeCubeCubeFlatSide.kThirdTrajectory,
                                        color)),
                        followTrajectory(
                                PathPlannerTrajectory.transformTrajectoryForAlliance(
                                        Trajectories.Blue.ConeCubeCubeFlatSide.kGetLastCube,
                                        color)),
                        // lock wheels so no slip
                        superstructure.drivetrainCommands.robotRelativeDrive(
                                new Translation2d(), FieldConstants.kZero, 0.3));
        // , getSupestructureSequenceConeCubeFlat()
        return Commands.parallel(
                drivetrainSequence,
                getSupestructureSequenceConeCubeCubeFlat(
                        Trajectories.Blue.ConeCubeBalanceFlatSide.kFirstTrajectory
                                .getTotalTimeSeconds(),
                        Trajectories.Blue.ConeCubeBalanceFlatSide.kSecondTrajectory
                                .getTotalTimeSeconds(),
                        Trajectories.Blue.ConeCubeBalanceFlatSide.kGoToBalance
                                .getTotalTimeSeconds(),
                        SuperstructureState.stowAll));
    }

    public Command justScore(SuperstructureState state, SuperstructureState nextState) {
        return Commands.sequence(
                Commands.parallel(
                        superstructure.goToStateParallel(state),
                        superstructure.rollersCommands.outCone().withTimeout(1)),
                superstructure.scoreAndStowConeReversed(nextState));
    }

    public Command justScore(SuperstructureState state) {
        return justScore(state, SuperstructureState.stowScore);
    }

    public Command justScoreBalance(SuperstructureState state, Alliance alliance) {
        var drivetrainSequence =
                Commands.sequence(
                        Commands.waitSeconds(5),
                        followTrajectory(
                                PathPlannerTrajectory.transformTrajectoryForAlliance(
                                        Trajectories.Blue.ConeBalance.kFirstTrajectory, alliance)),
                        Commands.run(
                                        () -> drive.drive(new Translation2d(1, 0), 0, false, true),
                                        drive)
                                .until(() -> Math.abs(drive.getPitch()) < 10)
                                .withTimeout(5),
                        superstructure
                                .drivetrainCommands
                                .robotRelativeDrive(
                                        new Translation2d(), Rotation2d.fromDegrees(5), 0.3)
                                .withTimeout(0.2));
        return Commands.parallel(drivetrainSequence, justScore(state));
    }

    public AutonomousMode getJustScoreBlue(SuperstructureState state) {
        return new AutonomousMode(justScore(state), new Pose2d(1.8, 3.26, FieldConstants.kZero));
    }

    public AutonomousMode getJustScoreRed(SuperstructureState state) {
        return new AutonomousMode(
                justScore(state), flipForPP(new Pose2d(1.8, 3.26, FieldConstants.kZero)));
    }

    public Command followTrajectory(PathPlannerTrajectory trajectory) {
        return new PPSwerveControllerCommand(
                trajectory,
                drive::getOdoPose,
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
