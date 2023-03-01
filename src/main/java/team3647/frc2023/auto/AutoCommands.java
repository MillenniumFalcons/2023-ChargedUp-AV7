package team3647.frc2023.auto;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import team3647.frc2023.constants.AutoConstants;
import team3647.frc2023.subsystems.Superstructure;
import team3647.frc2023.subsystems.SwerveDrive;
import team3647.frc2023.util.SuperstructureState;

public class AutoCommands {
    private final SwerveDrive drive;
    private final SwerveDriveKinematics driveKinematics;
    private final Superstructure superstructure;

    public final Blue blue = new Blue();
    public final Red red = new Red();

    public AutoCommands(
            SwerveDrive drive,
            SwerveDriveKinematics driveKinematics,
            Superstructure superstructure) {
        this.drive = drive;
        this.driveKinematics = driveKinematics;
        this.superstructure = superstructure;
    }

    private Command getSupestructureSequenceTwoPieces() {
        return Commands.sequence(
                superstructure.goToStateParallel(SuperstructureState.coneThreeReversed),
                superstructure.scoreAndStow(0.5).withTimeout(1.25),
                new WaitCommand(0),
                superstructure.groundIntake(),
                new WaitCommand(1),
                Commands.parallel(
                                superstructure.grabberCommands.closeGrabber(),
                                Commands.waitSeconds(0.5).andThen(superstructure.stow()))
                        .withTimeout(0.6),
                new WaitCommand(3),
                superstructure.goToStateParallel(SuperstructureState.cubeThreeReversed),
                superstructure.scoreAndStow(0.5));
    }

    private Command getSupestructureSequenceThreePieces() {
        return getSupestructureSequenceTwoPieces()
                .andThen(new WaitCommand(0.5))
                .andThen(superstructure.groundIntake())
                .andThen(new WaitCommand(1))
                .andThen(
                        Commands.parallel(
                                superstructure.grabberCommands.closeGrabber(),
                                Commands.waitSeconds(0.5).andThen(superstructure.stow())))
                .andThen(new WaitCommand(2))
                .andThen(superstructure.goToStateParallel(SuperstructureState.coneThree))
                .andThen(superstructure.scoreAndStow(0.5));
    }

    public class Blue {

        public Command rightSideConeCube() {
            Command drivetrainSequence =
                    Commands.sequence(
                            new WaitCommand(1.75), // Wait to score cone
                            followTrajectory(Trajectories.Blue.rightSideConeCubeFirst, false),
                            new WaitCommand(1.3), // close grabber
                            followTrajectory(Trajectories.Blue.rightSideConeCubeSecond, false),
                            new WaitCommand(1.5), // score cube
                            followTrajectory(Trajectories.Blue.rightGoToBalance, false));

            return Commands.parallel(drivetrainSequence, getSupestructureSequenceTwoPieces());
        }

        public Command leftSideConeCube() {
            // Command drivetrainSequence =
            //         Commands.sequence(
            //                 new WaitCommand(1), // Wait to score cone
            //                 followTrajectory(Trajectories.Blue.leftSide, false),
            //                 new WaitCommand(0.5), // close grabber
            //                 followTrajectory(Trajectories.Blue.leftSideConeCubeSecond, false),
            //                 new WaitCommand(1), // score cube
            //                 followTrajectory(Trajectories.Blue.leftGoToBalance, false));

            return Commands.parallel(getSupestructureSequenceTwoPieces());
        }

        public Command justScore(SuperstructureState state) {
            return Commands.sequence(
                    superstructure.goToStateParallel(state),
                    Commands.waitSeconds(0.5),
                    superstructure.scoreAndStow(0.5));
        }

        private Blue() {}
    }

    public class Red {
        public Command rightSideConeCube() {
            Command drivetrainSequence =
                    Commands.sequence(
                            new WaitCommand(1), // Wait to score cone
                            followTrajectory(Trajectories.Red.rightSideConeCubeFirst, false),
                            new WaitCommand(0.5), // close grabber
                            followTrajectory(Trajectories.Red.rightSideConeCubeSecond, false),
                            new WaitCommand(1), // score cube
                            followTrajectory(Trajectories.Red.rightGoToBalance, false));

            return Commands.parallel(getSupestructureSequenceThreePieces());
        }

        public Command leftSideConeCube() {
            Command drivetrainSequence =
                    Commands.sequence(
                            new WaitCommand(1.75), // Wait to score cone
                            followTrajectory(Trajectories.Red.leftSideConeCubeFirst, false),
                            new WaitCommand(1.3), // close grabber
                            followTrajectory(Trajectories.Red.leftSideConeCubeSecond, false),
                            new WaitCommand(1.5), // score cube
                            followTrajectory(Trajectories.Red.leftGoToBalance, false));

            return Commands.parallel(drivetrainSequence, getSupestructureSequenceTwoPieces());
        }

        public Command justScore(SuperstructureState state) {
            return Commands.sequence(
                    superstructure.goToStateParallel(state), superstructure.scoreAndStow(0.5));
        }

        private Red() {}
    }

    public Command followTrajectory(PathPlannerTrajectory trajectory, boolean firstPath) {
        return new SequentialCommandGroup(
                Commands.runOnce(
                        () -> {
                            // reset odometry
                            if (firstPath) {
                                drive.setRobotPose(trajectory.getInitialHolonomicPose());
                            }
                        }),
                new PPSwerveControllerCommand(
                        trajectory,
                        drive::getOdoPose,
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
