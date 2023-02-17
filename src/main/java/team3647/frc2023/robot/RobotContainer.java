package team3647.frc2023.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import team3647.frc2023.constants.ColorSensorConstants;
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
import team3647.lib.NetworkColorSensor;
import team3647.lib.inputs.ControlPanel;
import team3647.lib.inputs.ControlPanel.Buttons;
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
        scheduler.registerSubsystem(
                swerve, printer, pivot, extender, grabber, visionController, scoreStateFinder);

        configureDefaultCommands();
        configureButtonBindings();
        configureSmartDashboardLogging();
        pivot.setEncoder(PivotConstants.kInitialAngle);
        extender.setEncoder(ExtenderConstants.kMinimumPositionMeters);
        swerve.setRobotPose(new Pose2d(12.75, 4.3, Rotation2d.fromDegrees(0)));
    }

    private void configureButtonBindings() {
        mainController.buttonX.whileTrue(
                superstructure
                        .drivetrainCommands
                        .balance(
                                SwerveDriveConstants.kPitchController,
                                SwerveDriveConstants.kRollController)
                        .until(mainController::anyStickMoved));
        mainController.buttonY.onTrue(
                Commands.run(() -> {}, pivot)
                        .withTimeout(0.5)
                        .alongWith(Commands.run(() -> {}, extender).withTimeout(0.5)));
        // left bumper intake
        // left trigger slow
        // right bumper release
        // right trigger auto drive
        mainController.rightBumper.whileTrue(superstructure.grabberCommands.openGrabber());

        mainController
                .leftBumper
                .onTrue(superstructure.loadingStation())
                .onFalse(
                        new WaitCommand(0.5)
                                .andThen(
                                        superstructure
                                                .drivetrainCommands
                                                .robotRelativeDrive(new Translation2d(-0.8, 0), 0.5)
                                                .until(mainController::anyStickMoved)));

        mainController.rightTrigger.onTrue(
                superstructure
                        .driveAndArm(
                                scoreStateFinder::getScorePoint, scoreStateFinder::getScoreLevel)
                        .alongWith(
                                new InstantCommand(
                                        () ->
                                                printer.addPose(
                                                        "target",
                                                        scoreStateFinder::getScorePose))));

        var leftStickYGreaterPoint15 =
                new Trigger(() -> Math.abs(coController.getLeftStickY()) > 0.15);

        leftStickYGreaterPoint15.onTrue(
                superstructure
                        .extenderCommands
                        .openLoopSlow(coController::getLeftStickY)
                        .until(leftStickYGreaterPoint15.negate().debounce(0.5)));
    }

    private void configureDefaultCommands() {
        swerve.setDefaultCommand(
                superstructure.drivetrainCommands.drive(
                        mainController::getLeftStickX,
                        mainController::getLeftStickY,
                        mainController::getRightStickX,
                        mainController::getLeftTriggerValue,
                        () -> true,
                        AllianceFlipUtil::shouldFlip));
        pivot.setDefaultCommand(
                superstructure
                        .pivotCommands
                        .setAngle(() -> PivotConstants.kInitialAngle)
                        .repeatedly());
        grabber.setDefaultCommand(superstructure.grabberCommands.closeGrabber());
        extender.setDefaultCommand(
                superstructure.extenderCommands.length(ExtenderConstants.kMinimumPositionMeters));
    }

    void configTestCommands() {
        Commands.run(() -> {}, extender).schedule();
        Commands.run(() -> {}, pivot).schedule();
        Commands.run(() -> {}, grabber).schedule();
    }

    public void setToCoast() {
        pivot.setToCoast();
        extender.setToCoast();
    }

    public double getPivotFFVoltage() {
        return PivotConstants.kG
                * (extender.getLengthMeters() - ExtenderConstants.kMinimumPositionMeters)
                / ExtenderConstants.kMaximumPositionMeters;
    }

    public void configureSmartDashboardLogging() {
        printer.addDouble("rot", swerve::getHeading);
        // printer.addPose("odo", swerve::getPose);
        printer.addPose("estim", swerve::getEstimPose);

        printer.addDouble("Pivot Deg", pivot::getAngle);
        printer.addDouble("Extender Ticks", extender::getNativePos);
        printer.addString("Game Piece", grabber::getGamePieceStr);

        printer.addBoolean("Column1 I guess", () -> coPanel.getButton(Buttons.Column1));
    }

    public Command getAutonomousCommand() {
        return Commands.none();
    }

    private final Joysticks mainController = new Joysticks(0);
    private final Joysticks coController = new Joysticks(1);
    private final ControlPanel coPanel = new ControlPanel(3);

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

    // right menu button cube, left menu button cone
    public final Grabber grabber =
            new Grabber(
                    GrabberConstants.pistons,
                    new NetworkColorSensor(
                            ColorSensorConstants.kProximityEntry,
                            ColorSensorConstants.kColorEntry,
                            ColorSensorConstants.kMaxReadDistance),
                    coController.rightMidButton,
                    coController.leftMidButton);

    public final Pivot pivot =
            new Pivot(
                    PivotConstants.kMaster,
                    PivotConstants.kSlave,
                    PivotConstants.kNativeVelToDPS,
                    PivotConstants.kNativePosToDegrees,
                    PivotConstants.kMinDegree,
                    PivotConstants.kMaxDegree,
                    PivotConstants.nominalVoltage,
                    this::getPivotFFVoltage,
                    GlobalConstants.kDt);

    public final Extender extender =
            new Extender(
                    ExtenderConstants.kMaster,
                    new SimpleMotorFeedforward(0, 0, 0),
                    ExtenderConstants.kNativeVelToMpS,
                    ExtenderConstants.kNativePosToMeters,
                    ExtenderConstants.kMinimumPositionMeters,
                    ExtenderConstants.kMaximumPositionMeters,
                    ExtenderConstants.nominalVoltage,
                    GlobalConstants.kDt);

    private final VisionController visionController =
            new VisionController(
                    new Limelight(
                            PhotonVisionConstants.kLimelightIP,
                            0,
                            LimelightConstant.kCamConstatnts),
                    swerve::addVisionMeasurement);

    private final Compressor compressor = new Compressor(GlobalConstants.kPCMType);

    private final PowerDistribution pdh = new PowerDistribution(1, ModuleType.kRev);
    final Superstructure superstructure =
            new Superstructure(swerve, pivot, extender, grabber, compressor);
    private final CommandScheduler scheduler = CommandScheduler.getInstance();
    final GroupPrinter printer = GroupPrinter.getInstance();
    final ScoreStateFinder scoreStateFinder =
            new ScoreStateFinder(
                    grabber::getGamepiece,
                    swerve::getEstimPose,
                    coController.buttonB,
                    coController.dPadDown,
                    coController.dPadUp);
}
