package team3647.frc2023.auto;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleSupplier;
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

    public final AutonomousMode blueConeCubeCubeBumpSideNoBump; // 3 piece bump side
    public final AutonomousMode blueConeCubeCubeFlatSideMode; // dont use
    public final AutonomousMode blueConeCubeCubeMidFlatSideMode; // 3 piece flat side
    public final AutonomousMode blueConeCubeBalanceFlatSideMode; // dont use
    public final AutonomousMode blueConeCubeBalanceBumpSideMode; // dont use
    public final AutonomousMode blueConeScoreExitBalance; // dont use
    public final AutonomousMode blueConeCubeBalance; // 2 piece and balance mid
    public final AutonomousMode blueConeBalance; // balance straight from cone
    public final AutonomousMode blueJustScore; // just score high cone
    public final AutonomousMode blueJustDrive; // just drive forward
    public final AutonomousMode
            blueConeCubeCubeLowBalanceBumpSide; // 3 piece and balance bump side wip
    public final AutonomousMode blueConeTaxiBalance; // over and back balance mid
    public final AutonomousMode blueConeCubeCubeHoldBalance; // 2.5 piece and balance bump side wip
    public final AutonomousMode blueConeCube2PieceBalance; // 2 piece and balance bump side

    public final AutonomousMode redConeCubeCubeBumpSideNoBump; // 3 piece bump side
    public final AutonomousMode redConeCubeCubeFlatSideMode; // dont use
    public final AutonomousMode redConeCubeCubeMidFlatSideMode; // 3 piece flat side
    public final AutonomousMode redConeCubeBalanceFlatSideMode; // dont use
    public final AutonomousMode redConeCubeBalanceBumpSideMode; // dont use
    public final AutonomousMode redConeScoreExitBalance; // dont use
    public final AutonomousMode redConeCubeBalance; // 2 piece and balance mid
    public final AutonomousMode redConeBalance; // balance straight from cone
    public final AutonomousMode redJustScore; // just score high cone
    public final AutonomousMode redJustDrive; // just drive forward
    public final AutonomousMode
            redConeCubeCubeLowBalanceBumpSide; // 3 piece and balance bump side wip
    public final AutonomousMode redConeTaxiBalance; // over and back balance mid
    public final AutonomousMode redConeCubeCubeHoldBalance; // 2.5 piece and balance bump side wip
    public final AutonomousMode redConeCube2PieceBalance; // 2 piece and balance bump side

    public AutoCommands(
            SwerveDrive drive,
            SwerveDriveKinematics driveKinematics,
            Superstructure superstructure) {
        this.drive = drive;
        this.driveKinematics = driveKinematics;
        this.superstructure = superstructure;
        Test = new AutonomousMode(drive(), Trajectories.Blue.Test.kStart);
        // blue side modes
        blueConeCubeCubeBumpSideNoBump =
                new AutonomousMode(
                        coneCubeCubeBumpSideNoBump(Alliance.Blue),
                        Trajectories.Blue.coneCubeCubeBumpSideNoBump.kFirstPathInitial);
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
        blueConeScoreExitBalance =
                new AutonomousMode(
                        justScoreExitBalance(SuperstructureState.coneThreeReversed, Alliance.Blue),
                        Trajectories.Blue.ConeExitBalance.kFirstPathInitial);
        blueConeCubeBalance =
                new AutonomousMode(
                        coneCubeBalance(), Trajectories.Blue.coneCubeBalance.kFirstPathInitial);
        blueConeBalance =
                new AutonomousMode(
                        coneBalance(), Trajectories.Blue.coneCubeBalance.kFirstPathInitial);
        blueJustScore =
                new AutonomousMode(
                        justScore(SuperstructureState.coneThreeReversed),
                        new Pose2d(0, 0, FieldConstants.kZero));
        blueConeCubeCubeMidFlatSideMode =
                new AutonomousMode(
                        coneCubeCubeMidBalanceFlatSide(Alliance.Blue),
                        Trajectories.Blue.ConeCubeCubeMidBalanceFlatSide.kFirstPathInitial);
        blueJustDrive =
                new AutonomousMode(
                        justDrive(Alliance.Blue), Trajectories.Blue.justDrive.kFirstPathInitial);
        blueConeCubeCubeLowBalanceBumpSide =
                new AutonomousMode(
                        coneCubeCubeLowBalanceBumpSideNoBump(Alliance.Blue),
                        Trajectories.Blue.coneCubeCubeBalanceBumpSideNoBump.kFirstPathInitial);
        blueConeTaxiBalance =
                new AutonomousMode(
                        coneTaxiBalance(),
                        Trajectories.Blue.coneCubeCubeBalanceBumpSideNoBump.kFirstPathInitial);
        blueConeCubeCubeHoldBalance =
                new AutonomousMode(
                        coneCubeCubeHoldBalanceBumpSideNoBump(Alliance.Blue),
                        Trajectories.Blue.coneCubeCubeHoldBalanceBumpSideNoBump.kFirstPathInitial);
        blueConeCube2PieceBalance =
                new AutonomousMode(
                        coneCube2PieceBalanceBumpSideNoBump(Alliance.Blue),
                        Trajectories.Blue.coneCube2PieceBalanceBumpSideNoBUmp.kFirstPathInitial);

        // red side modes
        redConeCubeCubeBumpSideNoBump =
                new AutonomousMode(
                        coneCubeCubeBumpSideNoBump(Alliance.Red),
                        flipForPP(Trajectories.Blue.coneCubeCubeBumpSideNoBump.kFirstPathInitial));
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

        redConeScoreExitBalance =
                new AutonomousMode(
                        justScoreExitBalance(SuperstructureState.coneThreeReversed, Alliance.Red),
                        flipForPP(Trajectories.Blue.ConeExitBalance.kFirstPathInitial));
        redConeCubeBalance =
                new AutonomousMode(
                        coneCubeBalance(),
                        flipForPP(Trajectories.Blue.coneCubeBalance.kFirstPathInitial));
        redConeBalance =
                new AutonomousMode(
                        coneBalance(),
                        flipForPP(Trajectories.Blue.coneCubeBalance.kFirstPathInitial));
        redJustScore =
                new AutonomousMode(
                        justScore(SuperstructureState.coneThreeReversed),
                        new Pose2d(0, 0, FieldConstants.kZero));
        redConeCubeCubeMidFlatSideMode =
                new AutonomousMode(
                        coneCubeCubeMidBalanceFlatSide(Alliance.Red),
                        flipForPP(
                                Trajectories.Blue.ConeCubeCubeMidBalanceFlatSide
                                        .kFirstPathInitial));
        redJustDrive =
                new AutonomousMode(
                        justDrive(Alliance.Red),
                        flipForPP(Trajectories.Blue.justDrive.kFirstPathInitial));
        redConeCubeCubeLowBalanceBumpSide =
                new AutonomousMode(
                        coneCubeCubeLowBalanceBumpSideNoBump(Alliance.Red),
                        flipForPP(
                                Trajectories.Blue.coneCubeCubeBalanceBumpSideNoBump
                                        .kFirstPathInitial));
        redConeTaxiBalance =
                new AutonomousMode(
                        coneTaxiBalance(),
                        flipForPP(
                                Trajectories.Blue.coneCubeCubeBalanceBumpSideNoBump
                                        .kFirstPathInitial));
        redConeCubeCubeHoldBalance =
                new AutonomousMode(
                        coneCubeCubeHoldBalanceBumpSideNoBump(Alliance.Red),
                        flipForPP(
                                Trajectories.Blue.coneCubeCubeHoldBalanceBumpSideNoBump
                                        .kFirstPathInitial));
        redConeCube2PieceBalance =
                new AutonomousMode(
                        coneCube2PieceBalanceBumpSideNoBump(Alliance.Red),
                        flipForPP(
                                Trajectories.Blue.coneCube2PieceBalanceBumpSideNoBUmp
                                        .kFirstPathInitial));
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

        return Commands.sequence(
                justScore(
                                SuperstructureState.coneThreeReversed,
                                SuperstructureState.stowAfterConeThreeReversed)
                        .raceWith(endRightAfterExtenderRetracted()),
                Commands.deadline(
                                superstructure.waitForCurrentSpike(12),
                                superstructure.goToStateParallel(
                                        SuperstructureState.longTongueCube),
                                superstructure.rollersCommands.openloop(() -> 0.8))
                        .withTimeout(2.8),
                superstructure.stow().withTimeout(0.5),
                Commands.waitSeconds(0.2),
                superstructure.goToStateParallel(SuperstructureState.cubeThreeReversed),
                superstructure
                        .scoreAndStowCube(0.2, -0.4, SuperstructureState.groundIntakeCube)
                        .raceWith(endRightAfterExtenderRetracted()),
                Commands.waitSeconds(1),
                Commands.deadline(
                                superstructure.waitForCurrentSpike(12),
                                superstructure.goToStateParallel(
                                        SuperstructureState.groundIntakeCube),
                                superstructure.rollersCommands.openloop(() -> 0.6))
                        .withTimeout(3),
                superstructure.goToStateParallel(nextState));
    }

    private Command getSupestructureSequenceConeCubeCubeMidFlat(
            double firstPathTime,
            double secondPathTime,
            double thirdPathTime,
            double fourthPathTime,
            SuperstructureState nextState) {

        return Commands.sequence(
                justScore(
                                SuperstructureState.coneThreeReversed,
                                SuperstructureState.stowAfterConeThreeReversed)
                        .raceWith(endRightAfterExtenderRetracted()),
                Commands.deadline(
                                superstructure.waitForCurrentSpike(12),
                                superstructure.goToStateParallel(
                                        SuperstructureState.longTongueCube),
                                superstructure.rollersCommands.openloop(() -> 0.8))
                        .withTimeout(2.8),
                superstructure.stow().withTimeout(0.5),
                Commands.waitSeconds(0.2),
                superstructure.goToStateParallel(SuperstructureState.cubeThreeReversed),
                superstructure
                        .scoreAndStowCube(0.2, -0.4, SuperstructureState.beforeLongTongueCube)
                        .raceWith(endRightAfterExtenderRetracted()),
                Commands.waitSeconds(1),
                Commands.deadline(
                                superstructure.waitForCurrentSpike(8),
                                superstructure.goToStateParallel(
                                        SuperstructureState.longTongueCube),
                                superstructure.rollersCommands.openloop(() -> 0.8))
                        .withTimeout(3.8),
                superstructure.stow().withTimeout(0.5),
                // Commands.waitSeconds(1),
                superstructure.goToStateParallel(SuperstructureState.cubeTwoReversedLoong),
                Commands.waitSeconds(0.3),
                superstructure
                        .scoreAndStowCube(0.3, -0.4, SuperstructureState.stowAll)
                        .raceWith(endRightAfterExtenderRetracted()),
                superstructure.goToStateParallel(nextState));
    }

    private Command getSuperstructureExitCone(SuperstructureState state) {
        return Commands.sequence(
                justScore(state, SuperstructureState.lowCG),
                Commands.waitSeconds(2.5),
                Commands.deadline(
                                superstructure.waitForCurrentSpike(8),
                                superstructure.goToStateParallel(
                                        SuperstructureState.groundIntakeCubeLong),
                                superstructure.rollersCommands.openloop(() -> 0.6))
                        .withTimeout(2),
                Commands.parallel(
                        superstructure.goToStateParallel(SuperstructureState.coneThrow),
                        Commands.waitSeconds(6)
                                .andThen(
                                        superstructure
                                                .rollersCommands
                                                .openloop(() -> -1)
                                                .withTimeout(0.4))));
    }

    private Command getSupestructureSequenceConeCubeBump() {
        return Commands.sequence(
                justScore(
                                SuperstructureState.coneThreeReversed,
                                SuperstructureState.stowAfterConeThreeReversed)
                        .raceWith(endRightAfterExtenderRetracted()),
                Commands.waitSeconds(0.3),
                Commands.deadline(
                                superstructure.waitForCurrentSpike(7),
                                superstructure.goToStateParallel(
                                        SuperstructureState.longTongueCube),
                                superstructure.rollersCommands.openloop(() -> 0.45))
                        .withTimeout(3),
                superstructure.stow().withTimeout(1.3),
                // Commands.waitSeconds(0.2),
                superstructure.goToStateParallel(SuperstructureState.cubeThreeReversed),
                // Commands.waitSeconds(0.5),
                superstructure.scoreAndStowCube(0.5, -0.4, SuperstructureState.stowScore),
                Commands.waitSeconds(0.5),
                Commands.deadline(
                                superstructure.waitForCurrentSpike(7),
                                superstructure.goToStateParallel(
                                        SuperstructureState.longTongueCube),
                                superstructure.rollersCommands.openloop(() -> 0.45))
                        .withTimeout(2.35),
                superstructure.stow().withTimeout(1.3),
                superstructure.goToStateParallel(SuperstructureState.cubeTwoReversed),
                Commands.waitSeconds(0.4),
                superstructure.scoreAndStowCube(0.5, -0.4, SuperstructureState.stowScore));
    }

    private Command getSuperstrcutreSequenceConeBalance() {
        return Commands.sequence(
                justScore(
                                SuperstructureState.coneThreeReversed,
                                SuperstructureState.stowAfterConeThreeReversed)
                        .raceWith(endRightAfterExtenderRetracted()),
                superstructure
                        .goToStateParallel(SuperstructureState.pushDownStation)
                        .withTimeout(2),
                Commands.waitSeconds(1),
                superstructure.stowScore(() -> SuperstructureState.stowScore));
    }

    private Command getSupeStructureSequenceConeTaxiBalance() {
        return Commands.sequence(
                justScore(
                                SuperstructureState.coneThreeReversed,
                                SuperstructureState.stowAfterConeThreeReversed)
                        .raceWith(endRightAfterExtenderRetracted()),
                // Commands.waitSeconds(0.5),
                superstructure
                        .goToStateParallel(SuperstructureState.pushDownStation)
                        .withTimeout(2),
                Commands.waitSeconds(1),
                superstructure.stowScore(() -> SuperstructureState.stowScore).withTimeout(1),
                Commands.waitSeconds(2),
                superstructure.goToStateParallel(SuperstructureState.untipReverse).withTimeout(2),
                Commands.waitSeconds(1),
                superstructure.goToStateParallel(SuperstructureState.backStow));
    }

    private Command getSupestructureSeqeunceConeCubeBalance() {
        return Commands.sequence(
                justScore(
                                SuperstructureState.coneThreeReversed,
                                SuperstructureState.stowAfterConeThreeReversed)
                        .raceWith(endRightAfterExtenderRetracted()),
                // Commands.waitSeconds(0.5),
                superstructure
                        .goToStateParallel(SuperstructureState.pushDownStation)
                        .withTimeout(2),
                Commands.waitSeconds(1),
                superstructure.stowScore(() -> SuperstructureState.stowScore).withTimeout(1),
                Commands.waitSeconds(1.5),
                Commands.deadline(
                                superstructure.waitForCurrentSpike(7),
                                superstructure.goToStateParallel(
                                        SuperstructureState.longTongueCube),
                                superstructure.rollersCommands.openloop(() -> 0.45))
                        .withTimeout(1.3),
                // superstructure.stow().withTimeout(1),
                superstructure.goToStateParallel(SuperstructureState.untipReverse).withTimeout(2),
                Commands.waitSeconds(1.2),
                // superstructure.stowScore().withTimeout(1),
                superstructure.goToStateParallel(SuperstructureState.cubeTwoReversed),
                // Commands.waitSeconds(1),
                // Commands.waitSeconds(0.5),
                superstructure.scoreAndStowCube(0.5, -0.4, SuperstructureState.stowScore),
                superstructure
                        .goToStateParallel(SuperstructureState.pushDownStation)
                        .withTimeout(2),
                Commands.waitSeconds(1),
                superstructure.stow().withTimeout(1));
    }

    private Command getSupestructureSequenceConeCubeCubeBumpNoBump() {
        return Commands.sequence(
                justScore(
                                SuperstructureState.coneThreeReversed,
                                SuperstructureState.stowAfterConeThreeReversed)
                        .raceWith(endRightAfterExtenderRetracted()),
                Commands.waitSeconds(0.2),
                Commands.deadline(
                                superstructure.waitForCurrentSpike(7),
                                superstructure.goToStateParallel(
                                        SuperstructureState.longTongueCube),
                                superstructure.rollersCommands.openloop(() -> 0.45))
                        .withTimeout(2.05),
                superstructure.stow().withTimeout(1),
                // Commands.waitSeconds(0.2),
                superstructure.goToStateParallel(SuperstructureState.cubeThreeReversed),
                // Commands.waitSeconds(0.5),
                superstructure.scoreAndStowCube(0.5, -0.4, SuperstructureState.stowScore),
                // Commands.waitSeconds(0.5),
                Commands.deadline(
                                superstructure.waitForCurrentSpike(7),
                                superstructure.goToStateParallel(
                                        SuperstructureState.longTongueCube),
                                superstructure.rollersCommands.openloop(() -> 0.45))
                        .withTimeout(3),
                superstructure.stow().withTimeout(1),
                superstructure.goToStateParallel(SuperstructureState.cubeTwoReversed),
                // Commands.waitSeconds(0.2),
                superstructure.scoreAndStowCube(0.5, -0.4, SuperstructureState.stowScore));
    }

    private Command getSupestructureSequenceConeCubeCubeBumpNoBumpBalance() {
        return Commands.sequence(
                justScore(
                                SuperstructureState.coneThreeReversed,
                                SuperstructureState.stowAfterConeThreeReversed)
                        .raceWith(endRightAfterExtenderRetracted()),
                Commands.waitSeconds(0.2),
                Commands.deadline(
                                superstructure.waitForCurrentSpike(7),
                                superstructure.goToStateParallel(
                                        SuperstructureState.longTongueCube),
                                superstructure.rollersCommands.openloop(() -> 0.45))
                        .withTimeout(2),
                superstructure.stow().withTimeout(1),
                // Commands.waitSeconds(0.2),
                superstructure.goToStateParallel(SuperstructureState.cubeThreeReversed),
                // Commands.waitSeconds(0.5),
                superstructure.scoreAndStowCube(0.5, -0.4, SuperstructureState.stowScore),
                // Commands.waitSeconds(0.5),
                Commands.deadline(
                                superstructure.waitForCurrentSpike(7),
                                superstructure.goToStateParallel(
                                        SuperstructureState.longTongueCube),
                                superstructure.rollersCommands.openloop(() -> 0.45))
                        .withTimeout(1.5),
                // superstructure.stow().withTimeout(1),
                superstructure.goToStateParallel(SuperstructureState.cubeTwoReversed),
                // Commands.waitSeconds(0.2),
                superstructure.scoreAndStowCube(0.5, -0.4, SuperstructureState.stowScore),
                superstructure.goToStateParallel(SuperstructureState.pushDownStation),
                Commands.waitSeconds(1),
                superstructure.stowScore(() -> SuperstructureState.stowScore));
    }

    private Command getSupestructureSequenceConeCube2PieceBumpNoBumpBalance() {
        return Commands.sequence(
                justScore(
                                SuperstructureState.coneThreeReversed,
                                SuperstructureState.stowAfterConeThreeReversed)
                        .raceWith(endRightAfterExtenderRetracted()),
                Commands.waitSeconds(0.2),
                Commands.deadline(
                                superstructure.waitForCurrentSpike(7),
                                superstructure.goToStateParallel(
                                        SuperstructureState.longTongueCube),
                                superstructure.rollersCommands.openloop(() -> 0.45))
                        .withTimeout(2),
                superstructure.stow().withTimeout(1),
                // Commands.waitSeconds(0.2),
                superstructure.goToStateParallel(SuperstructureState.cubeThreeReversed),
                // Commands.waitSeconds(0.5),
                superstructure.scoreAndStowCube(0.5, -0.4, SuperstructureState.stowScore),
                // Commands.waitSeconds(0.5),
                superstructure.goToStateParallel(SuperstructureState.pushDownStation),
                Commands.waitSeconds(1),
                superstructure.stowScore(() -> SuperstructureState.stowScore));
    }

    private Command getSupestructureSequenceConeCubeCubeHoldBumpNoBumpBalance() {
        return Commands.sequence(
                justScore(
                                SuperstructureState.coneThreeReversed,
                                SuperstructureState.stowAfterConeThreeReversed)
                        .raceWith(endRightAfterExtenderRetracted()),
                Commands.waitSeconds(0.2),
                Commands.deadline(
                                superstructure.waitForCurrentSpike(7),
                                superstructure.goToStateParallel(
                                        SuperstructureState.longTongueCube),
                                superstructure.rollersCommands.openloop(() -> 0.45))
                        .withTimeout(2),
                superstructure.stow().withTimeout(1),
                // Commands.waitSeconds(0.2),
                superstructure.goToStateParallel(SuperstructureState.cubeThreeReversed),
                // Commands.waitSeconds(0.5),
                superstructure.scoreAndStowCube(0.5, -0.4, SuperstructureState.stowScore),
                // Commands.waitSeconds(0.5),
                Commands.deadline(
                                superstructure.waitForCurrentSpike(7),
                                superstructure.goToStateParallel(
                                        SuperstructureState.longTongueCube),
                                superstructure.rollersCommands.openloop(() -> 0.45))
                        .withTimeout(1.5),
                superstructure.goToStateParallel(SuperstructureState.untipReverse),
                Commands.waitSeconds(1),
                superstructure.goToStateParallel(SuperstructureState.backStow));
    }

    private Command getSupestructureSequenceConeCubeCubeFlat() {
        return Commands.sequence(
                justScore(
                                SuperstructureState.coneThreeReversed,
                                SuperstructureState.stowAfterConeThreeReversed)
                        .raceWith(endRightAfterExtenderRetracted()),
                // Commands.waitSeconds(0.2),
                Commands.deadline(
                                superstructure.waitForCurrentSpike(7),
                                superstructure.goToStateParallel(
                                        SuperstructureState.longTongueCube),
                                superstructure.rollersCommands.openloop(() -> 0.45))
                        .withTimeout(2.2),
                superstructure.stow().withTimeout(0.5),
                // Commands.waitSeconds(0.2),
                superstructure.goToStateParallel(SuperstructureState.cubeThreeReversed),
                // Commands.waitSeconds(0.5),
                superstructure.scoreAndStowCube(0.5, -0.4, SuperstructureState.stowScore),
                // Commands.waitSeconds(0.5),
                Commands.deadline(
                                superstructure.waitForCurrentSpike(7),
                                superstructure.goToStateParallel(
                                        SuperstructureState.longTongueCube),
                                superstructure.rollersCommands.openloop(() -> 0.45))
                        .withTimeout(2.2),
                superstructure.stow().withTimeout(1),
                superstructure.goToStateParallel(SuperstructureState.cubeTwoReversed),
                Commands.waitSeconds(0.2),
                superstructure.scoreAndStowCube(0.5, -0.4, SuperstructureState.stowScore));
    }

    public Command drive() {
        return followTrajectory(Trajectories.Blue.Test.kTrajectory);
    }

    public Command justDrive(Alliance color) {
        Command drivetrainSequence =
                followTrajectory(
                        PathPlannerTrajectory.transformTrajectoryForAlliance(
                                Trajectories.Blue.justDrive.kFirstTrajectory, color));
        return drivetrainSequence;
    }

    public Command coneCubeCubeBumpSideNoBump(Alliance color) {
        Command drivetrainSequence =
                Commands.sequence(
                        Commands.waitSeconds(1.8), // score cone
                        followTrajectory(
                                PathPlannerTrajectory.transformTrajectoryForAlliance(
                                        Trajectories.Blue.coneCubeCubeBumpSideNoBump
                                                .kFirstTrajectory,
                                        color)),
                        // rollers don't need waiting
                        followTrajectory(
                                PathPlannerTrajectory.transformTrajectoryForAlliance(
                                        Trajectories.Blue.coneCubeCubeBumpSideNoBump
                                                .kSecondTrajectory,
                                        color)),
                        followTrajectory(
                                PathPlannerTrajectory.transformTrajectoryForAlliance(
                                        Trajectories.Blue.coneCubeCubeBumpSideNoBump
                                                .kThirdTrajectory,
                                        color)),
                        // followTrajectory(
                        //         PathPlannerTrajectory.transformTrajectoryForAlliance(
                        //                 Trajectories.Blue.ConeCubeBalanceBumpSide.kIntakeCube,
                        //                 color)),
                        followTrajectory(
                                PathPlannerTrajectory.transformTrajectoryForAlliance(
                                        Trajectories.Blue.coneCubeCubeBumpSideNoBump
                                                .kFourthTrajectory,
                                        color)));
        // ,
        return Commands.parallel(
                drivetrainSequence, getSupestructureSequenceConeCubeCubeBumpNoBump());
    }

    public Command coneCubeCubeLowBalanceBumpSideNoBump(Alliance color) {
        Command drivetrainSequence =
                Commands.sequence(
                        Commands.waitSeconds(1.8), // score cone
                        followTrajectory(
                                PathPlannerTrajectory.transformTrajectoryForAlliance(
                                        Trajectories.Blue.coneCubeCubeBalanceBumpSideNoBump
                                                .kFirstTrajectory,
                                        color)),
                        // rollers don't need waiting
                        followTrajectory(
                                PathPlannerTrajectory.transformTrajectoryForAlliance(
                                        Trajectories.Blue.coneCubeCubeBalanceBumpSideNoBump
                                                .kSecondTrajectory,
                                        color)),
                        followTrajectory(
                                PathPlannerTrajectory.transformTrajectoryForAlliance(
                                        Trajectories.Blue.coneCubeCubeBalanceBumpSideNoBump
                                                .kThirdTrajectory,
                                        color)),
                        // followTrajectory(
                        //         PathPlannerTrajectory.transformTrajectoryForAlliance(
                        //                 Trajectories.Blue.ConeCubeBalanceBumpSide.kIntakeCube,
                        //                 color)),
                        followTrajectory(
                                PathPlannerTrajectory.transformTrajectoryForAlliance(
                                        Trajectories.Blue.coneCubeCubeBalanceBumpSideNoBump
                                                .kFourthTrajectory,
                                        color)),
                        driveForwardSlwoly().until(() -> (drive.getPitch() < -13)),
                        driveForwardSlwoly().until(() -> (drive.getPitch() > -13)),
                        lock());
        // ,
        return Commands.parallel(
                drivetrainSequence, getSupestructureSequenceConeCubeCubeBumpNoBumpBalance());
    }

    public Command coneCube2PieceBalanceBumpSideNoBump(Alliance color) {
        Command drivetrainSequence =
                Commands.sequence(
                        Commands.waitSeconds(1.8), // score cone
                        followTrajectory(
                                PathPlannerTrajectory.transformTrajectoryForAlliance(
                                        Trajectories.Blue.coneCube2PieceBalanceBumpSideNoBUmp
                                                .kFirstTrajectory,
                                        color)),
                        // rollers don't need waiting
                        followTrajectory(
                                PathPlannerTrajectory.transformTrajectoryForAlliance(
                                        Trajectories.Blue.coneCube2PieceBalanceBumpSideNoBUmp
                                                .kSecondTrajectory,
                                        color)),
                        Commands.waitSeconds(0.8),
                        followTrajectory(
                                PathPlannerTrajectory.transformTrajectoryForAlliance(
                                        Trajectories.Blue.coneCube2PieceBalanceBumpSideNoBUmp
                                                .kThirdTrajectory,
                                        color)),
                        // followTrajectory(
                        //         PathPlannerTrajectory.transformTrajectoryForAlliance(
                        //                 Trajectories.Blue.ConeCubeBalanceBumpSide.kIntakeCube,
                        //                 color)),
                        driveForwardSlwoly().until(() -> (drive.getPitch() < -13)),
                        driveForwardSlwoly().until(() -> (drive.getPitch() > -13)),
                        lock());
        // ,
        return Commands.parallel(
                drivetrainSequence, getSupestructureSequenceConeCube2PieceBumpNoBumpBalance());
    }

    public Command coneCubeCubeHoldBalanceBumpSideNoBump(Alliance color) {
        Command drivetrainSequence =
                Commands.sequence(
                        Commands.waitSeconds(1.8), // score cone
                        followTrajectory(
                                PathPlannerTrajectory.transformTrajectoryForAlliance(
                                        Trajectories.Blue.coneCubeCubeHoldBalanceBumpSideNoBump
                                                .kFirstTrajectory,
                                        color)),
                        // rollers don't need waiting
                        followTrajectory(
                                PathPlannerTrajectory.transformTrajectoryForAlliance(
                                        Trajectories.Blue.coneCubeCubeHoldBalanceBumpSideNoBump
                                                .kSecondTrajectory,
                                        color)),
                        followTrajectory(
                                PathPlannerTrajectory.transformTrajectoryForAlliance(
                                        Trajectories.Blue.coneCubeCubeHoldBalanceBumpSideNoBump
                                                .kThirdTrajectory,
                                        color)),
                        Commands.waitSeconds(0.8),
                        // followTrajectory(
                        //         PathPlannerTrajectory.transformTrajectoryForAlliance(
                        //                 Trajectories.Blue.ConeCubeBalanceBumpSide.kIntakeCube,
                        //                 color)),
                        followTrajectory(
                                PathPlannerTrajectory.transformTrajectoryForAlliance(
                                        Trajectories.Blue.coneCubeCubeHoldBalanceBumpSideNoBump
                                                .kFourthTrajectory,
                                        color)),
                        driveBackwardSlowly().until(() -> (drive.getPitch() > 13)),
                        driveBackwardSlowly().until(() -> (drive.getPitch() < 13)),
                        lock());
        // ,
        return Commands.parallel(
                drivetrainSequence, getSupestructureSequenceConeCubeCubeHoldBumpNoBumpBalance());
    }

    public Command coneCubeBalanceBumpSide(Alliance color) {
        Command drivetrainSequence =
                Commands.sequence(
                        Commands.waitSeconds(1.8), // score cone
                        followTrajectory(
                                PathPlannerTrajectory.transformTrajectoryForAlliance(
                                        Trajectories.Blue.ConeCubeBalanceBumpSide.kFirstTrajectory,
                                        color)),
                        // rollers don't need waiting
                        followTrajectory(
                                PathPlannerTrajectory.transformTrajectoryForAlliance(
                                        Trajectories.Blue.ConeCubeBalanceBumpSide.kSecondTrajectory,
                                        color)),
                        followTrajectory(
                                PathPlannerTrajectory.transformTrajectoryForAlliance(
                                        Trajectories.Blue.ConeCubeBalanceBumpSide
                                                .kGoToOutsideAndIntake,
                                        color)),
                        // followTrajectory(
                        //         PathPlannerTrajectory.transformTrajectoryForAlliance(
                        //                 Trajectories.Blue.ConeCubeBalanceBumpSide.kIntakeCube,
                        //                 color)),
                        followTrajectory(
                                PathPlannerTrajectory.transformTrajectoryForAlliance(
                                        Trajectories.Blue.ConeCubeBalanceBumpSide.kShootFromBump,
                                        color)));
        // ,
        return Commands.parallel(drivetrainSequence, getSupestructureSequenceConeCubeBump());
    }

    public Command coneCubeBalanceFlatSide(Alliance color) {
        final Trigger closeToBalanced = new Trigger(() -> Math.abs(drive.getPitch()) < 10);
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
                                                        new Translation2d(-0.85, 0),
                                                        0,
                                                        false,
                                                        true,
                                                        false),
                                        drive)
                                .until(closeToBalanced.debounce(0.05))
                                .withTimeout(6),

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
                        SuperstructureState.lowCG));
    }

    public Command driveForward(DoubleSupplier rotation) {
        return Commands.run(
                () ->
                        drive.drive(
                                new Translation2d(1.2, 0),
                                rotation.getAsDouble(),
                                true,
                                true,
                                false),
                drive);
    }

    public Command driveForwardFast() {
        return Commands.run(
                () -> drive.drive(new Translation2d(1.8, 0), 0, true, true, false), drive);
    }

    public Command driveForward() {
        return driveForward(() -> 0);
    }

    public Command driveForwardSlwoly() {
        return Commands.run(
                () -> drive.drive(new Translation2d(0.8, 0), 0, true, true, false), drive);
    }

    public Command driveBackward(DoubleSupplier rotation) {
        return Commands.run(
                () ->
                        drive.drive(
                                new Translation2d(-1.2, 0),
                                rotation.getAsDouble(),
                                true,
                                true,
                                false),
                drive);
    }

    public Command driveBackward() {
        return driveBackward(() -> 0);
    }

    public Command driveBackwardSlowly() {
        return Commands.run(
                () -> drive.drive(new Translation2d(-0.8, 0), 0, true, true, false), drive);
    }

    public Command driveNowhere() {
        return Commands.run(
                () -> drive.drive(new Translation2d(0, 0), 0, true, true, false), drive);
    }

    public Command lock() {
        return Commands.run(
                        () ->
                                drive.drive(
                                        new Translation2d(0, 0), Math.PI / 2.0, false, true, false),
                        drive)
                .withTimeout(0.1);
    }

    public Command coneBalance() {
        Command drivetrainSequence =
                Commands.sequence(
                        Commands.waitSeconds(2.5),
                        // followTrajectory(
                        //
                        // PathPlannerTrajectory.transformTrajectoryForAlliance(
                        //
                        // Trajectories.Blue.coneCubeBalance.kFirstTrajectory,
                        //                         color))
                        //         .until(() -> (drive.getPitch() < -0.1)),
                        driveForwardSlwoly().until(() -> (drive.getPitch() < -13)),
                        driveForwardSlwoly().until(() -> (drive.getPitch() > -13)),
                        // rotateAndBalance(SwerveDriveConstants.kAutoSteerHeadingController),
                        lock());
        return Commands.parallel(drivetrainSequence, getSuperstrcutreSequenceConeBalance());
    }

    public Command coneCubeBalance() {
        Command drivetrainSequence =
                Commands.sequence(
                        Commands.waitSeconds(3),
                        // followTrajectory(
                        //
                        // PathPlannerTrajectory.transformTrajectoryForAlliance(
                        //
                        // Trajectories.Blue.coneCubeBalance.kFirstTrajectory,
                        //                         color))
                        //         .until(() -> (drive.getPitch() < -0.1)),
                        driveForward().until(() -> (drive.getPitch() > 5)),
                        driveForward().withTimeout(1.2),
                        driveNowhere().withTimeout(1.5),
                        driveBackward().until(() -> (drive.getPitch() < -5)),
                        driveBackward().withTimeout(1.25),
                        driveNowhere().withTimeout(1.2),
                        driveForwardSlwoly().until(() -> (drive.getPitch() < -13)),
                        driveForwardSlwoly().until(() -> (drive.getPitch() > -13)),
                        // rotateAndBalance(SwerveDriveConstants.kAutoSteerHeadingController),
                        lock());
        return Commands.parallel(drivetrainSequence, getSupestructureSeqeunceConeCubeBalance());
    }

    public Command coneTaxiBalance() {
        Command drivetrainSeqeuence =
                Commands.sequence(
                        Commands.waitSeconds(3),
                        driveForwardFast().until(() -> (drive.getPitch() > 5)),
                        driveForward().withTimeout(1.2),
                        driveBackwardSlowly().until(() -> (drive.getPitch() > 13)),
                        driveBackwardSlowly().until(() -> (drive.getPitch() < 13)),
                        lock().withTimeout(0.1));
        return Commands.parallel(drivetrainSeqeuence, getSupeStructureSequenceConeTaxiBalance());
    }

    public Command rotateAndBalance(PIDController controller) {
        return Commands.sequence(
                Commands.run(
                                () -> {
                                    controller.setTolerance(1);
                                    double motionTurnComponent = 0;
                                    double error = drive.getHeading();
                                    motionTurnComponent =
                                            Math.abs(error - 180) < 1
                                                    ? 0
                                                    : controller.calculate(error);
                                    drive.drive(
                                            new Translation2d(),
                                            motionTurnComponent,
                                            true,
                                            true,
                                            false);
                                },
                                drive)
                        .withTimeout(1),
                Commands.run(
                        () ->
                                drive.drive(
                                        new Translation2d(
                                                controller.calculate(drive.getPitch()), 0),
                                        0,
                                        true,
                                        true,
                                        false)));
    }

    public Command coneCubeCubeMidBalanceFlatSide(Alliance color) {
        Command drivetrainSequence =
                Commands.sequence(
                        Commands.waitSeconds(1.6), // score cone
                        followTrajectory(
                                PathPlannerTrajectory.transformTrajectoryForAlliance(
                                        Trajectories.Blue.ConeCubeCubeMidBalanceFlatSide
                                                .kFirstTrajectory,
                                        color)),
                        // intake rollers don't need waiting
                        followTrajectory(
                                PathPlannerTrajectory.transformTrajectoryForAlliance(
                                        Trajectories.Blue.ConeCubeCubeMidBalanceFlatSide
                                                .kSecondTrajectory,
                                        color)),
                        followTrajectory(
                                PathPlannerTrajectory.transformTrajectoryForAlliance(
                                        Trajectories.Blue.ConeCubeCubeMidBalanceFlatSide
                                                .kThirdTrajectory,
                                        color)),
                        followTrajectory(
                                PathPlannerTrajectory.transformTrajectoryForAlliance(
                                        Trajectories.Blue.ConeCubeCubeMidBalanceFlatSide
                                                .kFourthTrajectory,
                                        color)));

        return Commands.parallel(
                drivetrainSequence, getSupestructureSequenceConeCubeCubeFlat()
                // getSupestructureSequenceConeCubeCubeMidFlat(
                //         Trajectories.Blue.ConeCubeCubeMidBalanceFlatSide.kFirstTrajectory
                //                 .getTotalTimeSeconds(),
                //         Trajectories.Blue.ConeCubeCubeMidBalanceFlatSide.kSecondTrajectory
                //                 .getTotalTimeSeconds(),
                //         Trajectories.Blue.ConeCubeCubeMidBalanceFlatSide.kThirdTrajectory
                //                 .getTotalTimeSeconds(),
                //         Trajectories.Blue.ConeCubeCubeMidBalanceFlatSide.kFourthTrajectory
                //                 .getTotalTimeSeconds(),
                //         SuperstructureState.stowAll)
                );
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

    public Command justScoreExitBalance(SuperstructureState state, Alliance alliance) {
        final Trigger closeToBalanced = new Trigger(() -> Math.abs(drive.getPitch()) < 10.4);
        var drivetrainSequence =
                Commands.sequence(
                        Commands.waitSeconds(1.5),
                        followTrajectory(
                                PathPlannerTrajectory.transformTrajectoryForAlliance(
                                        Trajectories.Blue.ConeExitBalance.kFirstTrajectory,
                                        alliance)),
                        Commands.run(
                                        () ->
                                                drive.drive(
                                                        new Translation2d(-0.8, 0),
                                                        0,
                                                        false,
                                                        true,
                                                        false),
                                        drive)
                                .until(() -> Math.abs(drive.getPitch()) > 9)
                                .withTimeout(2),
                        Commands.run(
                                        () ->
                                                drive.drive(
                                                        new Translation2d(-0.5, 0),
                                                        0,
                                                        false,
                                                        true,
                                                        false),
                                        drive)
                                .until(closeToBalanced.debounce(0.05))
                                .withTimeout(5),
                        Commands.run(
                                        () ->
                                                drive.drive(
                                                        new Translation2d(0, 0),
                                                        Math.PI / 2.0,
                                                        false,
                                                        true,
                                                        false),
                                        drive)
                                .withTimeout(0.1),
                        Commands.run(
                                        () ->
                                                drive.drive(
                                                        new Translation2d(0, 0),
                                                        0,
                                                        false,
                                                        true,
                                                        false),
                                        drive)
                                .withTimeout(0.1));
        return Commands.parallel(drivetrainSequence, getSuperstructureExitCone(state));
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
