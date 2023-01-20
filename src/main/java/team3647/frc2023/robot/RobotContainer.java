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
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

import com.pathplanner.lib.PathPoint;
import team3647.frc2023.commands.AutoCommands;
import team3647.frc2023.commands.PathPlannerTrajectories;
import team3647.frc2023.commands.SwerveDriveNoAim;
import team3647.frc2023.constants.PhotonVisionConstants;
import team3647.frc2023.constants.SwerveDriveConstants;
import team3647.frc2023.subsystems.Superstructure;
import team3647.frc2023.subsystems.SwerveDrive;
import team3647.frc2023.subsystems.vision.LimelightController;
import team3647.frc2023.subsystems.vision.PhotonVisionCamera;
import team3647.lib.GroupPrinter;
import team3647.lib.inputs.Joysticks;
import team3647.lib.vision.Limelight;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    private final Joysticks mainController = new Joysticks(0);
    private Pose2d target = new Pose2d();
//     private final Joysticks coController = new Joysticks(1);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        scheduler.registerSubsystem(limelightController, m_printer, m_swerve);

        configureButtonBindings();
        configureDefaultCommands();
        configureSmartDashboardLogging();
        m_printer.addPose("Target Pose", () -> this.target);
        // m_swerve.setOdometry(
        //         PathPlannerTrajectories.spinStartPose, new Rotation2d(Units.degreesToRadians(180)));
        m_swerve.setOdometry(
                new Pose2d(5, 5, new Rotation2d(Units.degreesToRadians(180))), new Rotation2d(Units.degreesToRadians(180)));
    }
    private final PathPoint kOriginPoint = new PathPoint(new Translation2d(Units.inchesToMeters(23), Units.inchesToMeters(48)), new Rotation2d(Units.degreesToRadians(0)), new Rotation2d(Units.degreesToRadians(180)));

    private void configureButtonBindings() {
        mainController.buttonA.onTrue(new InstantCommand(() -> m_swerve.zeroHeading()));
        mainController.leftBumper.onTrue(new InstantCommand(m_swerve::extend));
        mainController.rightBumper.onTrue(new InstantCommand(m_swerve::retract));
        //mainController.buttonB.onTrue(Commands.runOnce(() -> this.target = getTargetPose()));
        mainController.buttonB.onTrue(new InstantCommand(() -> m_printer.addDouble("tag pose x", this::getTagPoseX)));
        mainController.buttonB.onTrue(new InstantCommand(() -> m_printer.addDouble("tag pose y", this::getTagPoseY))); 
        mainController.buttonB.onTrue(new InstantCommand(() -> m_printer.addDouble("target pose x", this::getTargetPoseX)));
        mainController.buttonB.onTrue(new InstantCommand(() -> m_printer.addDouble("target pose y", this::getTargetPoseY)));         
        mainController.buttonB.onTrue(
            new InstantCommand(
        () -> {    
        new PrintCommand("Starting!").andThen(m_swerve.getTrajectoryCommand(m_swerve.getToPointATrajectory(getCalculatedTargetPose(48)))
        .withTimeout(8)).schedule();}));
     }

     public Pose2d getTargetPose() {
        var offset = new Transform3d(new Translation3d(Units.inchesToMeters(-23), Units.inchesToMeters(-48), 0), new Rotation3d());
        var cameraToTagTransform = new Transform3d(limelightController.getCameraToTag(), new Rotation3d());
        var robotPose3d = new Pose3d(m_swerve.getPose());
        var fieldToTag = robotPose3d.transformBy(PhotonVisionConstants.robotToCam.times(-1.0)).transformBy(cameraToTagTransform);

        var pose3d = fieldToTag.transformBy(offset);

        return new Pose2d(new Translation2d(pose3d.getX(), pose3d.getY()), new Rotation2d());
     }

     public double getTargetPoseX() {
        return getTargetPose().getX();
     }

     public double getTargetPoseY() {
        return getTargetPose().getY();
     }

     public Pose2d getTagPose() {
        var cameraToTagTransform = new Transform3d(limelightController.getCameraToTag(), new Rotation3d());
        var robotPose3d = new Pose3d(m_swerve.getPose());
        var fieldToTag = robotPose3d.transformBy(PhotonVisionConstants.robotToCam).transformBy(cameraToTagTransform);

        var pose3d = fieldToTag;

        return new Pose2d(new Translation2d(pose3d.getX(), pose3d.getY()), new Rotation2d());
     }

     public double getTagPoseX() {
        return getTagPose().getX();
     }

     public double getTagPoseY() {
        return getTagPose().getY();
     }

     // left and right of tag (meters)
     private PathPoint getCalculatedTargetPose(double fromTag) {
        double cameraToTagX = limelightController.getCameraToTagX();
        double cameraToTagY = limelightController.getCameraToTagY();
        double robotToFlushX = 0;
        double robotToFlushY = 0;
        double tagOffsetDepth = PhotonVisionConstants.offsetAprilTagToCenterOfRobotFlush;
        robotToFlushX = cameraToTagX - tagOffsetDepth + 0.288 * Math.sin(-Units.degreesToRadians(m_swerve.getRawHeading()) - Math.atan(0.1 / 0.27));
        robotToFlushY = cameraToTagY + fromTag + 0.288 * Math.cos(-Units.degreesToRadians(m_swerve.getRawHeading()) - Math.atan(0.1 / 0.27));
        //robotToFlushX = cameraToTagX - tagOffsetDepth + Math.abs(Math.sin((Units.degreesToRadians(90 - m_swerve.getHeading()))) * PhotonVisionConstants.robotToCam.getX());
        //robotToFlushY = cameraToTagY + tagOffsetSideway + Math.abs(Math.cos(Units.degreesToRadians(m_swerve.getHeading())) * PhotonVisionConstants.robotToCam.getY());
        double xSetPoint = m_swerve.getPose().getX() - robotToFlushX;
        double ySetPoint = m_swerve.getPose().getY() + robotToFlushY;
        SmartDashboard.putNumber("x", robotToFlushX);
        SmartDashboard.putNumber("x set point", xSetPoint);
        SmartDashboard.putNumber("y", robotToFlushY);
        SmartDashboard.putNumber("y set point", ySetPoint);
        SmartDashboard.putNumber("robot flush y", robotToFlushY);
        SmartDashboard.putNumber("robot flush x", robotToFlushX);
        return new PathPoint(new Translation2d(xSetPoint, ySetPoint), new Rotation2d(0), new Rotation2d(Units.degreesToRadians(180)));
        // var cameraToTagTransform = new Transform3d(limelightController.getCameraToTag(), new Rotation3d());
        // var robotPose3d = new Pose3d(m_swerve.getPose());
        // var fromTag3d = new Transform3d(new Translation3d(fromTag.getX(), fromTag.getY(), 0), new Rotation3d());
        // var fieldToTag = robotPose3d.transformBy(PhotonVisionConstants.robotToCam).transformBy(cameraToTagTransform);

        // var pose3d = fieldToTag.transformBy(fromTag3d);

        // return new PathPoint(new Translation2d(pose3d.getX(), pose3d.getY()), new Rotation2d(), Rotation2d.fromDegrees(180));
        // var cameraToTagTransform = new Transform3d(limelightController.getCameraToTag(), new Rotation3d());
        // var robotPose3d = new Pose3d(m_swerve.getPose());
        // var fromTag3d = new Transform3d(new Translation3d(fromTag.getX(), fromTag.getY(), 0), new Rotation3d());
        // var fieldToTag = robotPose3d.transformBy(PhotonVisionConstants.robotToCam).transformBy(cameraToTagTransform);
        // var pose3d = fieldToTag.transformBy(fromTag3d);

        // return new PathPoint(new Translation2d(pose3d.getX(), pose3d.getY()), new Rotation2d(), Rotation2d.fromDegrees(180));
     }

    //  private Pose2d getTargetPose() {
    //     Translation2d fromTag = new Translation2d(1, 1);
    //     var cameraToTagTransform = photonVisionCamera.getCameraToTagTransform();
    //     var robotPose3d = new Pose3d(m_swerve.getPose());
        
    //     var fromTag3d = new Transform3d(new Translation3d(fromTag.getX(), fromTag.getY(), 0), new Rotation3d());
    //     var fieldToTag = robotPose3d.transformBy(PhotonVisionConstants.robotToCam).transformBy(cameraToTagTransform);

    //     var pose3d = fieldToTag.transformBy(fromTag3d);

    //     return new Pose2d(new Translation2d(pose3d.getX(), pose3d.getY()), new Rotation2d());
    //  }

    //  private PathPoint getZeroPath() {
    //     Translation2d currentTranslation = new Translation2d(m_swerve.getPose().getX(), m_swerve.getPose().getY());
    //     return new PathPoint(currentTranslation, new Rotation2d(0), new Rotation2d(Units.degreesToRadians(180)));
    //  }

    private void configureDefaultCommands() {
        m_swerve.setDefaultCommand(
                new SwerveDriveNoAim(
                        m_swerve,
                        mainController::getLeftStickX,
                        mainController::getLeftStickY,
                        mainController::getRightStickX,
                        () -> true));
    }

    private final LimelightController limelightController = new LimelightController();

    public void configureSmartDashboardLogging() {
        // m_printer.addDouble("rot", m_swerve::getRawHeading);
        m_printer.addDouble("robot to tag x", limelightController::getCameraToTagX);
        m_printer.addDouble("robot to tag y", limelightController::getCameraToTagY);
        m_printer.addDouble("robot to tag rot", limelightController::getYaw);
        m_printer.addDouble("robot x", m_swerve::getPoseX);
        m_printer.addDouble("robot y", m_swerve::getPoseY);
    }

    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        // return autoCommands.getPathCommand();
        return null;
    }

    // private final PhotonVisionCamera photonVisionCamera =
    //                 new PhotonVisionCamera(PhotonVisionConstants.camera);

    public final SwerveDrive m_swerve =
            new SwerveDrive(
                    SwerveDriveConstants.kFrontLeftModule,
                    SwerveDriveConstants.kFrontRightModule,
                    SwerveDriveConstants.kBackLeftModule,
                    SwerveDriveConstants.kBackRightModule,
                    SwerveDriveConstants.kGyro
                    );

    // private final AutoCommands autoCommands = new AutoCommands(this.m_swerve);
    

    private final CommandScheduler scheduler = CommandScheduler.getInstance();

    private final GroupPrinter m_printer = GroupPrinter.getInstance();
}
