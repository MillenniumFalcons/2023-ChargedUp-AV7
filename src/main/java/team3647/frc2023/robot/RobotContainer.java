package team3647.frc2023.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import team3647.frc2023.commands.SwerveDriveNoAim;
import team3647.frc2023.constants.LimelightConstant;
import team3647.frc2023.constants.PhotonVisionConstants;
import team3647.frc2023.constants.SwerveDriveConstants;
import team3647.frc2023.subsystems.Superstructure;
import team3647.frc2023.subsystems.SwerveDrive;
import team3647.frc2023.subsystems.VisionController;
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

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        scheduler.registerSubsystem(swerve, printer, visionController);

        configureButtonBindings();
        configureDefaultCommands();
        configureSmartDashboardLogging();

        swerve.setOdometry(
                new Pose2d(2, 2, Rotation2d.fromDegrees(180)), Rotation2d.fromDegrees(180));
    }

    private void configureButtonBindings() {
        mainController.buttonA.onTrue(Commands.runOnce(swerve::zeroHeading));

        mainController.buttonX.whileTrue(
                superstructure
                        .drivetrainCommands
                        .balance(
                                SwerveDriveConstants.kPitchController,
                                SwerveDriveConstants.kRollController)
                        .until(mainController::anyStickMoved));
    }

    private void configureDefaultCommands() {
        swerve.setDefaultCommand(
                new SwerveDriveNoAim(
                        swerve,
                        mainController::getLeftStickX,
                        mainController::getLeftStickY,
                        mainController::getRightStickX,
                        () -> true));
    }

    public void configureSmartDashboardLogging() {
        printer.addDouble("rot", swerve::getRawHeading);
        printer.addDouble("robot pitch", swerve::getRoll);
        printer.addPose("robot pose", swerve::getPose);
        printer.addPose("ESTIMATED", swerve::getEstimPose);
        printer.addDouble("Joystick", mainController::getLeftStickY);
    }

    public Command getAutonomousCommand() {
        return Commands.none();
    }

    private final Joysticks mainController = new Joysticks(0);
    private final Joysticks coController = new Joysticks(1);

    public final SwerveDrive swerve =
            new SwerveDrive(
                    SwerveDriveConstants.kFrontLeftModule,
                    SwerveDriveConstants.kFrontRightModule,
                    SwerveDriveConstants.kBackLeftModule,
                    SwerveDriveConstants.kBackRightModule,
                    SwerveDriveConstants.kGyro);

    private final VisionController visionController =
            new VisionController(
                    new Limelight(
                            PhotonVisionConstants.kLimelightIP,
                            "limelight",
                            0,
                            LimelightConstant.kCamConstatnts),
                    swerve::addVisionMeasurment);

    private final Superstructure superstructure = new Superstructure(swerve);

    private final CommandScheduler scheduler = CommandScheduler.getInstance();

    private final GroupPrinter printer = GroupPrinter.getInstance();
}
