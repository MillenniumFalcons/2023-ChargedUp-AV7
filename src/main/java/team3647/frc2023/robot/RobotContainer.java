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

import java.util.ArrayList;

import org.photonvision.targeting.PhotonTrackedTarget;

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
    private Pose2d target = new Pose2d();
//     private final Joysticks coController = new Joysticks(1);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        scheduler.registerSubsystem(
                m_swerve, photonVisionCamera,
                m_printer);

        configureButtonBindings();
        configureDefaultCommands();
        configureSmartDashboardLogging();
        // m_swerve.setOdometry(
        //         PathPlannerTrajectories.spinStartPose, new Rotation2d(Units.degreesToRadians(180)));
        m_swerve.setOdometry(
                new Pose2d(2, 2, new Rotation2d(Units.degreesToRadians(180))), new Rotation2d(Units.degreesToRadians(180)));
    }
    private final PathPoint kOriginPoint = new PathPoint(new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(0)), new Rotation2d(Units.degreesToRadians(0)), new Rotation2d(Units.degreesToRadians(180)));

    private void configureButtonBindings() {
        mainController.buttonA.onTrue(new InstantCommand(() -> m_swerve.zeroHeading()));
        mainController.buttonB.onTrue(Commands.runOnce(() -> this.target = getTargetPose()));
        mainController.buttonB.onTrue(new InstantCommand(() -> getTagPose()));
        mainController.buttonB.onTrue(new InstantCommand(() -> m_printer.addPose("target pose", () -> target)));
        mainController.buttonB.onTrue(
            new InstantCommand(
        () -> {    
        new PrintCommand("Starting!").andThen(m_swerve.getTrajectoryCommand(m_swerve.getToPointATrajectory(getCalculatedTargetPose(new Translation2d(1, 1))))
        .withTimeout(8)).schedule();}));
     }

     public void getTagPose() {
        if (photonVisionCamera.getHasTarget()) {
            for (PhotonTrackedTarget target : photonVisionCamera.getAllTargets()) {
                var cameraToTagTransform = target.getBestCameraToTarget();
                var robotPose3d = new Pose3d(m_swerve.getPose());
                var fieldToTag = robotPose3d.transformBy(PhotonVisionConstants.robotToCam).transformBy(cameraToTagTransform);
                var fieldToTag2d = new Pose2d(new Translation2d(fieldToTag.getX(), fieldToTag.getY()), new Rotation2d(fieldToTag.getRotation().getAngle()));

                String id = target.getFiducialId() + "";
                m_printer.addPose(id, () -> fieldToTag2d);
            }        
        }
     }

     // left and right of tag (meters)
     // should weight with ambiguity for average
     private PathPoint getCalculatedTargetPose(Translation2d fromTag) {
        ArrayList<Pose3d> calculatedPoses = new ArrayList<Pose3d>();
        if (photonVisionCamera.getHasTarget()) {
            for (PhotonTrackedTarget target : photonVisionCamera.getAllTargets()) {
                var pose3d = new Pose3d();
                // ensure target is good, higher than 0.2 ambiguity is garbage
                if (target.getFiducialId() == 3 && target.getPoseAmbiguity() < 0.15) {
                    var cameraToTagTransform = target.getBestCameraToTarget();
                    var robotPose3d = new Pose3d(m_swerve.getPose());
                    var fromTag3d = new Transform3d(new Translation3d(fromTag.getX(), fromTag.getY(), 0), new Rotation3d());
                    var fieldToTag = robotPose3d.transformBy(PhotonVisionConstants.robotToCam).transformBy(cameraToTagTransform);
                    pose3d = fieldToTag.transformBy(fromTag3d);
                } 
    
                if (target.getFiducialId() == 2 && target.getPoseAmbiguity() < 0.15) {
                    var cameraToTagTransform = target.getBestCameraToTarget();
                    var robotPose3d = new Pose3d(m_swerve.getPose());
                    // negative y for to the left since its a different tag pose
                    var fromTag3d = new Transform3d(new Translation3d(fromTag.getX(), -fromTag.getY(), 0), new Rotation3d());
                    var fieldToTag = robotPose3d.transformBy(PhotonVisionConstants.robotToCam).transformBy(cameraToTagTransform);
                    pose3d = fieldToTag.transformBy(fromTag3d);
                }
                calculatedPoses.add(pose3d);
            }
        }
            
        double sumX  = 0;
        double sumY = 0;
        for (Pose3d calculatedPose : calculatedPoses) {
            sumX += calculatedPose.getX();
            sumY += calculatedPose.getY();
        }

        double avgX = (sumX * 1.0) / calculatedPoses.size();
        double avgY = (sumY * 1.0) / calculatedPoses.size();

        Translation2d avgTargetPose = new Translation2d(avgX, avgY);
        
        return new PathPoint(new Translation2d(avgTargetPose.getX(), avgTargetPose.getY()), new Rotation2d(), Rotation2d.fromDegrees(180));
     }

     private Pose2d getTargetPose() {
        Translation2d fromTag = new Translation2d(1, 1);
        ArrayList<Pose3d> calculatedPoses = new ArrayList<Pose3d>();
        if (photonVisionCamera.getHasTarget()) {
            for (PhotonTrackedTarget target : photonVisionCamera.getAllTargets()) {
                var pose3d = new Pose3d();
                if (target.getFiducialId() == 3) {
                    var cameraToTagTransform = target.getBestCameraToTarget();
                    var robotPose3d = new Pose3d(m_swerve.getPose());
                    var fromTag3d = new Transform3d(new Translation3d(fromTag.getX(), fromTag.getY(), 0), new Rotation3d());
                    var fieldToTag = robotPose3d.transformBy(PhotonVisionConstants.robotToCam).transformBy(cameraToTagTransform);
                    pose3d = fieldToTag.transformBy(fromTag3d);
                } 
    
                if (target.getFiducialId() == 2) {
                    var cameraToTagTransform = target.getBestCameraToTarget();
                    var robotPose3d = new Pose3d(m_swerve.getPose());
                    // negative y for to the left since its a different tag pose
                    var fromTag3d = new Transform3d(new Translation3d(fromTag.getX(), -fromTag.getY(), 0), new Rotation3d());
                    var fieldToTag = robotPose3d.transformBy(PhotonVisionConstants.robotToCam).transformBy(cameraToTagTransform);
                    pose3d = fieldToTag.transformBy(fromTag3d);
                }
                calculatedPoses.add(pose3d);
            }
        }
            
        double sumX  = 0;
        double sumY = 0;
        for (Pose3d calculatedPose : calculatedPoses) {
            sumX += calculatedPose.getX();
            sumY += calculatedPose.getY();
        }

        double avgX = (sumX * 1.0) / calculatedPoses.size();
        double avgY = (sumY * 1.0) / calculatedPoses.size();

        Translation2d avgTargetPose = new Translation2d(avgX, avgY);
        
        return new Pose2d(avgTargetPose.getX(), avgTargetPose.getY(), Rotation2d.fromDegrees(180));
     }

    //  public Pose2d getTagPoseLimelight() {
    //     Translation2d fromTag = new Translation2d(Units.inchesToMeters(-23), Units.inchesToMeters(48));
    //     var cameraToTagTransform = new Transform3d(limelightController.getCameraToTag(), new Rotation3d());
    //     var robotPose3d = new Pose3d(m_swerve.getPose());
    //     var fromTag3d = new Transform3d(new Translation3d(fromTag.getX(), fromTag.getY(), 0), new Rotation3d());
    //     var fieldToTag = robotPose3d.transformBy(PhotonVisionConstants.robotToCam).transformBy(cameraToTagTransform);

    //     var pose3d = fieldToTag.transformBy(fromTag3d);

    //     return new Pose2d(pose3d.getX(), pose3d.getY(), new Rotation2d());
    //  }

     // left and right of tag (meters)
    //  private PathPoint getCalculatedTargetPoseLimelight(double fromTag) {

    //     double cameraToTagX = limelightController.getCameraToTagX();
    //     double cameraToTagY = limelightController.getCameraToTagY();
    //     double robotToFlushX = 0;
    //     double robotToFlushY = 0;
    //     double tagOffsetDepth = PhotonVisionConstants.offsetAprilTagToCenterOfRobotFlush;
    //     robotToFlushX = cameraToTagX - tagOffsetDepth + 0.295 * Math.sin(Units.degreesToRadians(m_swerve.getRawHeading() - 180) + Math.atan(0.1 / 0.27));
    //     robotToFlushY = cameraToTagY + fromTag + 0.295 * Math.cos(Units.degreesToRadians(m_swerve.getRawHeading() - 180) + Math.atan(0.1 / 0.27));
    //     //robotToFlushX = cameraToTagX - tagOffsetDepth + Math.abs(Math.sin((Units.degreesToRadians(90 - m_swerve.getHeading()))) * PhotonVisionConstants.robotToCam.getX());
    //     //robotToFlushY = cameraToTagY + tagOffsetSideway + Math.abs(Math.cos(Units.degreesToRadians(m_swerve.getHeading())) * PhotonVisionConstants.robotToCam.getY());
    //     double xSetPoint = m_swerve.getPose().getX() - robotToFlushX;
    //     double ySetPoint = m_swerve.getPose().getY() + robotToFlushY;
    //     // SmartDashboard.putNumber("x", robotToFlushX);
    //     // SmartDashboard.putNumber("x set point", xSetPoint);
    //     // SmartDashboard.putNumber("y", robotToFlushY);
    //     // SmartDashboard.putNumber("y set point", ySetPoint);
    //     // SmartDashboard.putNumber("robot flush y", robotToFlushY);
    //     // SmartDashboard.putNumber("robot flush x", robotToFlushX);
    //     return new PathPoint(new Translation2d(xSetPoint, ySetPoint), new Rotation2d(0), new Rotation2d(Units.degreesToRadians(180)));
    //     // var cameraToTagTransform = new Transform3d(limelightController.getCameraToTag(), new Rotation3d());
    //     // var robotPose3d = new Pose3d(m_swerve.getPose());
    //     // var fromTag3d = new Transform3d(new Translation3d(fromTag.getX(), fromTag.getY(), 0), new Rotation3d());
    //     // var fieldToTag = robotPose3d.transformBy(PhotonVisionConstants.robotToCam).transformBy(cameraToTagTransform);
    //     // var pose3d = fieldToTag.transformBy(fromTag3d);
    //     // m_printer.addDouble("calculated x", pose3d::getX);
    //     // m_printer.addDouble("calculated y", pose3d::getY);

    //     // return new PathPoint(new Translation2d(pose3d.getX(), pose3d.getY()), new Rotation2d(), Rotation2d.fromDegrees(180));
    //  }

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
        m_printer.addPose("robot pose", m_swerve::getPose);
        m_printer.addPose("ESTIMATED", m_swerve::getEstimPose);
        m_printer.addDouble("Joystick", mainController::getLeftStickY);
    }

    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return null;
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

    private final CommandScheduler scheduler = CommandScheduler.getInstance();

    private final GroupPrinter m_printer = GroupPrinter.getInstance();
}
