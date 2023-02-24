package team3647.frc2023.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import team3647.frc2023.commands.AutoCommands;
import team3647.frc2023.commands.PathPlannerTrajectories;
import team3647.frc2023.constants.ExtenderConstants;
import team3647.frc2023.constants.GlobalConstants;
import team3647.frc2023.constants.GrabberConstants;
import team3647.frc2023.constants.LimelightConstant;
import team3647.frc2023.constants.PivotConstants;
import team3647.frc2023.constants.SwerveDriveConstants;
import team3647.frc2023.subsystems.Extender;
import team3647.frc2023.subsystems.Grabber;
import team3647.frc2023.subsystems.Pivot;
import team3647.frc2023.subsystems.Superstructure;
import team3647.frc2023.subsystems.SwerveDrive;
import team3647.frc2023.subsystems.VisionController;
import team3647.frc2023.subsystems.VisionController.CAMERA_NAME;
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
                swerve,
                printer,
                pivot,
                extender,
                grabber,
                visionController2,
                visionController3,
                panelScoreStateFinder);
        // scheduler.registerSubsystem(rollerGrabber);

        configureDefaultCommands();
        configureButtonBindings();
        configureSmartDashboardLogging();
        pivot.setEncoder(PivotConstants.kInitialAngle);
        extender.setEncoder(ExtenderConstants.kMinimumPositionTicks);
        // swerve.setRobotPose(new Posse2d(12.75, 4.3, Rotation2d.fromDegrees(0)));
        swerve.setRobotPose(PathPlannerTrajectories.topS_P4.getInitialPose());
    }

    private void configureButtonBindings() {
        mainController.buttonX.whileTrue(
                superstructure
                        .drivetrainCommands
                        .balance(
                                SwerveDriveConstants.kPitchController,
                                SwerveDriveConstants.kRollController)
                        .until(mainController::anyStickMoved));

        mainController.buttonA.onTrue(
                superstructure
                        .grabberCommands
                        .openGrabber()
                        .withTimeout(0.7)
                        .andThen(superstructure.stow()));

        mainController
                .leftBumper
                .whileTrue(
                        superstructure
                                .armToGroundIntake()
                                .alongWith(superstructure.grabberCommands.openGrabber()))
                .onFalse(
                        superstructure
                                .grabberCommands
                                .closeGrabber()
                                .withTimeout(0.5)
                                .andThen(superstructure.stow()));

        // left trigger slow
        // right bumper release
        // right trigger auto drive

        // hold and line up, release and wait for it to drive back
        // mainController
        //         .rightBumper
        //         .onTrue(
        //                 superstructure
        //                         .grabberCommands
        //                         .openGrabber()
        //                         .alongWith(superstructure.doubleStation()))
        //         .onFalse(
        //                 superstructure
        //                         .grabberCommands
        //                         .closeGrabber()
        //                         .withTimeout(0.5)
        //                         .andThen(
        //                                 superstructure.drivetrainCommands.robotRelativeDrive(
        //                                         new Translation2d(-0.8, 0), 0.5))
        //                         .until(mainController::anyStickMoved));

        mainController
                .rightBumper
                .onTrue(
                        superstructure
                                .grabberCommands
                                .openGrabber()
                                .alongWith(superstructure.doubleStation()))
                .onFalse(
                        superstructure
                                .grabberCommands
                                .closeGrabber()
                                .withTimeout(0.5)
                                .andThen(superstructure.stow())
                                .until(mainController::anyStickMoved));

        mainController.buttonY.onTrue(
                Commands.run(() -> {}, grabber)
                        .alongWith(Commands.run(() -> {}, extender))
                        .alongWith(Commands.run(() -> {}, pivot))
                        .withTimeout(0.2)
                        .andThen(
                                superstructure.driveAndArmSequential(
                                        panelScoreStateFinder::getScorePoint,
                                        panelScoreStateFinder::getScoreLevel))
                        .alongWith(
                                new InstantCommand(
                                        () ->
                                                printer.addPose(
                                                        "target",
                                                        panelScoreStateFinder::getScorePose))));
        mainController
                .rightTrigger
                .whileTrue(
                        new InstantCommand(
                                        () ->
                                                visionController.changePipeline(
                                                        LimelightConstant.TAPE_PIPELINE))
                                .withTimeout(0.5)
                                .andThen(
                                        superstructure.drivetrainCommands.rotateToTape(
                                                SwerveDriveConstants.kYController,
                                                visionController::getXToTape,
                                                0.05)))
                .onFalse(
                        new InstantCommand(
                                () ->
                                        visionController.changePipeline(
                                                LimelightConstant.APRIL_PIPELINE)));

        // manual arm scoring
        var manualLevel = new Trigger(() -> ctrlPanelOverrides.getWhiteOne());
        manualLevel.onTrue(superstructure.arm(() -> panelScoreStateFinder.getScoreLevel()));

        var stowButton = new Trigger(() -> ctrlPanelOverrides.getRedThree());
        stowButton.onTrue(superstructure.stow());

        var extenderOverrideOnForward =
                new Trigger(
                        () ->
                                ctrlPanelScoring.getJoystickFBAxis() > 0.5
                                        && ctrlPanelOverrides.getRedFour());

        extenderOverrideOnForward.whileTrue(
                superstructure
                        .extenderCommands
                        .openloop(() -> -0.15)
                        .until(extenderOverrideOnForward.negate().debounce(0.5)));

        var extenderOverrideOnReverse =
                new Trigger(
                        () ->
                                ctrlPanelScoring.getJoystickFBAxis() < -0.5
                                        && ctrlPanelOverrides.getRedFour());

        extenderOverrideOnReverse.whileTrue(
                superstructure
                        .extenderCommands
                        .openloop(() -> 0.15)
                        .until(extenderOverrideOnReverse.negate().debounce(0.5)));

        var pivotOverrideOnFoward =
                new Trigger(
                        () ->
                                ctrlPanelOverrides.getJoystickFBAxis() > 0.5
                                        && ctrlPanelOverrides.getRedFour());

        pivotOverrideOnFoward.whileTrue(
                superstructure
                        .pivotCommands
                        .openloop(() -> 0.09)
                        .until(pivotOverrideOnFoward.negate().debounce(0.5)));

        var pivotOverrideOnReverse =
                new Trigger(
                        () ->
                                ctrlPanelOverrides.getJoystickFBAxis() < -0.5
                                        && ctrlPanelOverrides.getRedFour());

        pivotOverrideOnReverse.whileTrue(
                superstructure
                        .pivotCommands
                        .openloop(() -> -0.09)
                        .until(pivotOverrideOnReverse.negate().debounce(0.5)));
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
        // grabber.setDefaultCommand(
        //         new ConditionalCommand(
        //                 superstructure.grabberCommands.closeGrabber(),
        //                 superstructure.grabberCommands.closeAndRollIn(),
        //                 grabber::getHasGamePiece));
        grabber.setDefaultCommand(superstructure.grabberCommands.closeGrabber());
        extender.setDefaultCommand(superstructure.extenderCommands.holdPositionAtCall());
    }

    void configTestCommands() {
        Commands.run(() -> {}, extender).schedule();
        Commands.run(() -> {}, pivot).schedule();
        Commands.run(() -> {}, grabber).schedule();
    }

    public void setToCoast() {
        // pivot.setToCoast();
        // extender.setToCoast();
    }

    public double getPivotFFVoltage() {
        return PivotConstants.kG
                * (extender.getNativeTicks() - ExtenderConstants.kMinimumPositionTicks)
                / ExtenderConstants.kMaximumPositionTicks;
    }

    public void configureSmartDashboardLogging() {
        printer.addDouble("rot", swerve::getHeading);
        printer.addPose("odo", swerve::getPose);
        printer.addPose("estim", swerve::getEstimPose);

        printer.addDouble("Pivot Deg", pivot::getAngle);
        printer.addDouble("Extender Ticks", extender::getNativePos);

        printer.addBoolean("Column1 I guess", () -> ctrlPanelScoring.getLevelLow());
        printer.addPose("target", panelScoreStateFinder::findScorePose);
        printer.addString("Level", panelScoreStateFinder::getScoreLevelStr);

        printer.addDouble("X Dis", visionController::getXToTape);

        SmartDashboard.putNumber("CENTER XY", 0.9);
        SmartDashboard.putNumber("CENTER ROT", 0.9);

        SmartDashboard.putNumber("LEFT XY", 0.9);
        SmartDashboard.putNumber("LEFT ROT", 0.9);

        SmartDashboard.putNumber("RIGHT XY", 0.9);
        SmartDashboard.putNumber("RIGHT ROT", 0.9);

        SmartDashboard.putNumber("OffsetX", 0);
        SmartDashboard.putNumber("OffsetY", 0);
    }

    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return autoCommands.top1C1B();
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
                    ctrlPanelScoring::getColumnNine,
                    swerve::getEstimPose);

    // right menu button cube, left menu button cone
    public final Grabber grabber = new Grabber(GrabberConstants.pistons);

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
                    ExtenderConstants.resetSensor,
                    new SimpleMotorFeedforward(0, 0, 0),
                    ExtenderConstants.kNativeVelToMpS,
                    ExtenderConstants.kNativePosToMeters,
                    ExtenderConstants.kMinimumPositionTicks,
                    ExtenderConstants.kMaximumPositionTicks,
                    ExtenderConstants.nominalVoltage,
                    GlobalConstants.kDt);

    private final VisionController visionController =
            new VisionController(
                    new Limelight(
                            LimelightConstant.kLimelightCenterIP,
                            LimelightConstant.kLimelightCenterHost,
                            0,
                            LimelightConstant.kCamConstatnts),
                    swerve::addVisionMeasurement,
                    CAMERA_NAME.CENTER);

    private final VisionController visionController2 =
            new VisionController(
                    new Limelight(
                            LimelightConstant.kLimelightLeftIP,
                            LimelightConstant.kLimelightLeftHost,
                            0,
                            LimelightConstant.kCamConstatnts),
                    swerve::addVisionMeasurement,
                    CAMERA_NAME.LEFT);

    private final VisionController visionController3 =
            new VisionController(
                    new Limelight(
                            LimelightConstant.kLimelightRightIP,
                            LimelightConstant.kLimelightRightHost,
                            0,
                            LimelightConstant.kCamConstatnts),
                    swerve::addVisionMeasurement,
                    CAMERA_NAME.RIGHT);

    private final Compressor compressor = new Compressor(GlobalConstants.kPCMType);

    private final PowerDistribution pdh = new PowerDistribution(1, ModuleType.kRev);
    final Superstructure superstructure =
            new Superstructure(swerve, pivot, extender, grabber, compressor);
    private final CommandScheduler scheduler = CommandScheduler.getInstance();
    final GroupPrinter printer = GroupPrinter.getInstance();
    private final AutoCommands autoCommands =
            new AutoCommands(swerve, SwerveDriveConstants.kDriveKinematics, superstructure);
}
