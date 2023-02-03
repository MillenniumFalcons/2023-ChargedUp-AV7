package team3647.frc2023.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import team3647.frc2023.constants.ExtenderConstants;
import team3647.frc2023.constants.GlobalConstants;
import team3647.frc2023.constants.GrabberConstants;
import team3647.frc2023.constants.LimelightConstant;
import team3647.frc2023.constants.PhotonVisionConstants;
import team3647.frc2023.constants.PivotConstants;
import team3647.frc2023.constants.SwerveDriveConstants;
import team3647.frc2023.subsystems.Extender;
import team3647.frc2023.subsystems.Grabber;
import team3647.frc2023.subsystems.Pivot;
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
        pdh.clearStickyFaults();
        scheduler.registerSubsystem(swerve, printer, pivot, extender); // visionController);

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
        mainController.buttonB.onTrue(superstructure.grabberCommands.setAngle(165));
        mainController.buttonY.onTrue(superstructure.grabberCommands.setAngle(0));
    }

    private void configureDefaultCommands() {
        swerve.setDefaultCommand(
                superstructure.drivetrainCommands.drive(
                        mainController::getLeftStickX,
                        mainController::getLeftStickY,
                        mainController::getRightStickX,
                        () -> true));
        // TODO delete later
        pivot.setDefaultCommand(superstructure.setPivotAngle(this::getPivotOutput));
        // extender.setDefaultCommand(
        //         superstructure.extenderCommands.openloop(coController::getRightStickY));
    }

    public void configureSmartDashboardLogging() {
        printer.addDouble("rot", swerve::getRawHeading);
        printer.addDouble("robot roll", swerve::getRoll);
        printer.addDouble("robot pitch", swerve::getPitch);
        printer.addPose("robot pose", swerve::getPose);
        printer.addPose("ESTIMATED", swerve::getEstimPose);
        printer.addDouble("Joystick", mainController::getLeftStickY);
        printer.addDouble("Pivot Deg", pivot::getAngle);
        printer.addDouble("Extender Distance", extender::getLengthMeters);
        printer.addDouble("Grabber Deg", grabber::getAngle);
        SmartDashboard.putNumber("Pivot", 0);
    }

    public double getPivotOutput() {
        return SmartDashboard.getNumber("Pivot", 0);
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
                    SwerveDriveConstants.kGyro,
                    SwerveDriveConstants.kDriveKinematics,
                    SwerveDriveConstants.kDrivePossibleMaxSpeedMPS,
                    SwerveDriveConstants.kRotPossibleMaxSpeedRadPerSec);

    public final Grabber grabber =
            new Grabber(
                    GrabberConstants.kMaster,
                    GrabberConstants.feedforward,
                    GrabberConstants.kNativeVelToDPS,
                    GrabberConstants.kNativePosToDegrees,
                    GrabberConstants.nominalVoltage,
                    GlobalConstants.kDt);

    public final Pivot pivot =
            new Pivot(
                    PivotConstants.kMaster,
                    PivotConstants.kSlave,
                    PivotConstants.kNativeVelToDPS,
                    PivotConstants.kNativePosToDegrees,
                    PivotConstants.nominalVoltage,
                    PivotConstants.kG,
                    GlobalConstants.kDt);

    public final Extender extender =
            new Extender(
                    ExtenderConstants.kMaster,
                    new SimpleMotorFeedforward(0, 0, 0),
                    ExtenderConstants.kNativeVelToMpS,
                    ExtenderConstants.kNativePosToMeters,
                    ExtenderConstants.nominalVoltage,
                    GlobalConstants.kDt);

    private final VisionController visionController =
            new VisionController(
                    new Limelight(
                            PhotonVisionConstants.kLimelightIP,
                            "limelight",
                            0,
                            LimelightConstant.kCamConstatnts),
                    swerve::addVisionMeasurment);

    private final PowerDistribution pdh = new PowerDistribution(1, ModuleType.kRev);

    private final Superstructure superstructure =
            new Superstructure(swerve, pivot, extender, grabber);

    private final CommandScheduler scheduler = CommandScheduler.getInstance();

    private final GroupPrinter printer = GroupPrinter.getInstance();
}
