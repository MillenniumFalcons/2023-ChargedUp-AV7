package team3647.frc2023.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
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
                PathPlannerTrajectories.spinStartPose, new Rotation2d(Units.degreesToRadians(180)));
    }
    private final PathPoint kOriginPoint = new PathPoint(new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(0)), new Rotation2d(Units.degreesToRadians(0)), new Rotation2d(Units.degreesToRadians(180)));

    private void configureButtonBindings() {
        mainController.buttonA.onTrue(new InstantCommand(() -> m_swerve.zeroHeading()));
        mainController.buttonB.onTrue(
            new InstantCommand(
        () -> {    
        new PrintCommand("Starting!").andThen(m_swerve.getTrajectoryCommand(m_swerve.getToPointATrajectory(getCalculatedTargetPose(new Translation2d(1, 1))))
        .withTimeout(8)).schedule();}).until(() -> {return mainController.getLeftStickX() != 0 || mainController.getLeftStickY() != 0 || mainController.getRightStickX() != 0;}));
     }

     public Pose2d getTagPose() {
        var cameraToTagTransform = photonVisionCamera.getCameraToTagTransform();
        var robotPose3d = new Pose3d(m_swerve.getPose());
        
        var fieldToTag = robotPose3d.transformBy(PhotonVisionConstants.robotToCam).transformBy(cameraToTagTransform);
        var fieldToTag2d = new Pose2d(new Translation2d(fieldToTag.getX(), fieldToTag.getY()), new Rotation2d(fieldToTag.getRotation().getAngle()));
        return fieldToTag2d;
     }

     // left and right of tag (meters)
     private PathPoint getCalculatedTargetPose(Translation2d fromTag) {
        var cameraToTagTransform = photonVisionCamera.getCameraToTagTransform();
        var robotPose3d = new Pose3d(m_swerve.getPose());
        
        var fromTag3d = new Transform3d(new Translation3d(fromTag.getX(), fromTag.getY(), 0), new Rotation3d());
        var fieldToTag = robotPose3d.transformBy(PhotonVisionConstants.robotToCam).transformBy(cameraToTagTransform);

        var pose3d = fieldToTag.transformBy(fromTag3d);

        return new PathPoint(new Translation2d(pose3d.getX(), pose3d.getY()), new Rotation2d(), Rotation2d.fromDegrees(180));
     }

     private PathPoint getZeroPath() {
        Translation2d currentTranslation = new Translation2d(m_swerve.getPose().getX(), m_swerve.getPose().getY());
        return new PathPoint(currentTranslation, new Rotation2d(0), new Rotation2d(Units.degreesToRadians(180)));
     }

    private void configureDefaultCommands() {
        m_swerve.setDefaultCommand(
                new SwerveDriveNoAim(
                        m_swerve,
                        mainController::getLeftStickX,
                        mainController::getLeftStickY,
                        mainController::getRightStickX,
                        () -> true));
    }

    public void configureSmartDashboardLogging() {
        m_printer.addDouble("rot", m_swerve::getRawHeading);
        m_printer.addPose("tag pose", this::getTagPose);
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
