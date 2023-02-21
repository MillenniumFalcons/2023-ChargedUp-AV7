package team3647.frc2023.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
import team3647.lib.inputs.ControlPanel;
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
                swerve, printer, pivot, extender, grabber, visionController, panelScoreStateFinder);
        // scheduler.registerSubsystem(rollerGrabber);

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

        mainController.buttonA.onTrue(superstructure.stow());
        mainController
                .leftBumper
                .whileTrue(superstructure.grabberCommands.openGrabber())
                .onFalse(superstructure.stow());
        // left bumper intake
        // left trigger slow
        // right bumper release
        // right trigger auto drive

        // hold and line up, release and wait for it to drive back
        mainController
                .rightBumper
                .onTrue(
                        superstructure
                                .grabberCommands
                                .openGrabber()
                                .alongWith(superstructure.loadingStation()))
                .onFalse(
                        superstructure
                                .grabberCommands
                                .closeGrabber()
                                .withTimeout(0.5)
                                .until(mainController::anyStickMoved));
        mainController.rightTrigger.onTrue(
                superstructure
                        .driveAndArmSequential(
                                panelScoreStateFinder::getScorePoint,
                                panelScoreStateFinder::getScoreLevel)
                        .alongWith(
                                new InstantCommand(
                                        () ->
                                                printer.addPose(
                                                        "target",
                                                        panelScoreStateFinder::getScorePose))));

        var groundIntakeButton = new Trigger(() -> ctrlPanelOverrides.getRedThree());

        groundIntakeButton
                .whileTrue(
                        superstructure
                                .groundIntake()
                                .alongWith(superstructure.grabberCommands.openGrabber()))
                .onFalse(
                        superstructure
                                .grabberCommands
                                .closeGrabber()
                                .alongWith(new WaitCommand(0.7).andThen(superstructure.stow())));

        var extenderOverrideOnForward =
                new Trigger(
                        () ->
                                Math.abs(ctrlPanelScoring.getJoystickFBAxis()) > 0.2
                                        && ctrlPanelOverrides.getRedFour());

        extenderOverrideOnForward.onTrue(
                superstructure
                        .extenderCommands
                        .openloop(() -> 0.2)
                        .until(extenderOverrideOnForward.negate().debounce(0.5)));

        var extenderOverrideOnReverse =
                new Trigger(
                        () ->
                                Math.abs(ctrlPanelScoring.getJoystickFBAxis()) < -0.2
                                        && ctrlPanelOverrides.getRedFour());

        extenderOverrideOnReverse.onTrue(
                superstructure
                        .extenderCommands
                        .openloop(() -> -0.2)
                        .until(extenderOverrideOnForward.negate().debounce(0.5)));

        var pivotOverrideOnFoward =
                new Trigger(
                        () ->
                                Math.abs(ctrlPanelOverrides.getJoystickFBAxis()) > 0.2
                                        && ctrlPanelOverrides.getRedFour());

        pivotOverrideOnFoward.onTrue(
                superstructure
                        .extenderCommands
                        .openloop(() -> 0.15)
                        .until(pivotOverrideOnFoward.negate().debounce(0.5)));

        var pivotOverrideOnReverse =
                new Trigger(
                        () ->
                                Math.abs(ctrlPanelOverrides.getJoystickFBAxis()) < -0.2
                                        && ctrlPanelOverrides.getRedFour());

        pivotOverrideOnReverse.onTrue(
                superstructure
                        .extenderCommands
                        .openloop(() -> -0.15)
                        .until(pivotOverrideOnFoward.negate().debounce(0.5)));
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
        pivot.setDefaultCommand(superstructure.pivotCommands.holdPositionAtCall());
        grabber.setDefaultCommand(superstructure.grabberCommands.closeGrabber());
        extender.setDefaultCommand(superstructure.extenderCommands.holdPositionAtCall());
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

        printer.addBoolean("Column1 I guess", () -> ctrlPanelScoring.getLevelLow());
        printer.addPose("target", panelScoreStateFinder::findScorePose);
        printer.addString("Level", panelScoreStateFinder::getScoreLevelStr);
        printer.addDouble("Value JOYSTICK Extender", ctrlPanelScoring::getJoystickFBAxis);
        printer.addDouble("Value JOYSTICK pivot", ctrlPanelOverrides::getJoystickFBAxis);
        printer.addBoolean("ButtonSSS", ctrlPanelOverrides::getRedFour);
    }

    public Command getAutonomousCommand() {
        return Commands.none();
    }

    private final Joysticks mainController = new Joysticks(0);
    private final ControlPanel ctrlPanelScoring = new ControlPanel(1);
    private final ControlPanel ctrlPanelOverrides = new ControlPanel(2);

    //     public final RollerGrabber rollerGrabber =
    //             new RollerGrabber(
    //                     RollerGrabberConstants.kMaster, RollerGrabberConstants.kPiston, 1, 1, 1,
    // 1);

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
                    GrabberConstants.kMaster,
                    GrabberConstants.pistons,
                    GrabberConstants.gamePieceSensor,
                    GrabberConstants.kNativeVelToMpS,
                    GrabberConstants.kNativePosToMeters,
                    GrabberConstants.kNominalVoltage,
                    GlobalConstants.kDt);

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
    final PanelScoreStateFinder panelScoreStateFinder =
            new PanelScoreStateFinder(
                    ctrlPanelOverrides::getLevelLow,
                    ctrlPanelScoring::getLevelMid,
                    ctrlPanelScoring::getLevelHigh,
                    ctrlPanelScoring::getColumnOne,
                    ctrlPanelScoring::getColumnTwo,
                    ctrlPanelScoring::getColumnThree,
                    ctrlPanelScoring::getColumnFour,
                    ctrlPanelScoring::getColumnFive,
                    ctrlPanelScoring::getColumnSix,
                    ctrlPanelScoring::getColumnSeven,
                    ctrlPanelScoring::getColumnEight,
                    ctrlPanelScoring::getColumnNine);
}
