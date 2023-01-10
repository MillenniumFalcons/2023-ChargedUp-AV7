package team3647.frc2023.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;

// import edu.wpi.first.wpilibj2.command.InstantCommand;
import java.util.LinkedList;
import java.util.List;

import com.pathplanner.lib.PathPoint;

import team3647.frc2023.commands.AutoCommands;
import team3647.frc2023.commands.PathPlannerTrajectories;
import team3647.frc2023.commands.SwerveDriveNoAim;
import team3647.frc2023.constants.PhotonVisionConstants;
import team3647.frc2023.constants.SwerveDriveConstants;
import team3647.frc2023.subsystems.Superstructure;
import team3647.frc2023.subsystems.SwerveDrive;
import team3647.frc2023.subsystems.vision.PhotonVisionCamera;
import team3647.lib.GroupPrinter;
import team3647.lib.inputs.Joysticks;
// import team3647.lib.tracking.RobotTracker;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    private final Joysticks mainController = new Joysticks(0);
//     private final Joysticks coController = new Joysticks(1);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        scheduler.registerSubsystem(
                m_swerve, photonVisionCamera,
                m_printer);

        configureButtonBindings();
        configureDefaultCommands();
        configureSmartDashboardLogging();
        // chooseAuto();
        // rot 2d is the rotation of the robot relative to field during auto
        // m_swerve.setOdometry(startPosition, startPosition.getRotation());

        m_swerve.setOdometry(
                PathPlannerTrajectories.spinStartPose,
                new Rotation2d(Units.degreesToRadians(0)));
        // m_swerve.setOdometry(
        //         PathPlannerTrajectories.startStateSixBallBump1,
        //         new Rotation2d(Units.degreesToRadians(45)));
    }
    private final PathPoint kOriginPoint = new PathPoint(new Translation2d(), new Rotation2d());
    private void configureButtonBindings() {
        mainController.buttonA.onTrue(new InstantCommand(() -> m_swerve.zeroHeading()));
        mainController.buttonB.onTrue(
            new InstantCommand(
        () -> {    
        new PrintCommand("Starting!").andThen(m_swerve.getTrajectoryCommand(m_swerve.getToPointATrajectory(kOriginPoint)).withTimeout(5)).schedule();}).until(() -> {return mainController.getLeftStickX() != 0 || mainController.getLeftStickY() != 0 || mainController.getRightStickX() != 0;}));
     }

    private void configureDefaultCommands() {
        m_swerve.setDefaultCommand(
                new SwerveDriveNoAim(
                        m_swerve,
                        mainController::getLeftStickX,
                        mainController::getLeftStickY,
                        mainController::getRightStickX,
                        // () -> new Pose2d(),
                        () -> true));
    }

    public void configureSmartDashboardLogging() {
        m_printer.addPose("Robot", m_swerve::getPose);
        m_printer.addPose("ESTIMTATED POSE", m_swerve::getEstimPose);
        // m_printer.addDouble("intake velocity brr", () -> m_intake.getVelocity()););
    }

    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return autoCommands.getPathCommand();
    }

    private final PhotonVisionCamera photonVisionCamera =
                    new PhotonVisionCamera(PhotonVisionConstants.camera);

    public final SwerveDrive m_swerve =
            new SwerveDrive(
                    SwerveDriveConstants.kFrontLeftModule,
                    SwerveDriveConstants.kFrontRightModule,
                    SwerveDriveConstants.kBackLeftModule,
                    SwerveDriveConstants.kBackRightModule,
                    SwerveDriveConstants.kGyro,
                    photonVisionCamera);

    final Superstructure m_superstructure =
            new Superstructure(m_swerve);

    private final AutoCommands autoCommands = new AutoCommands(this.m_swerve);

    private final CommandScheduler scheduler = CommandScheduler.getInstance();

    private final GroupPrinter m_printer = GroupPrinter.getInstance();
}
