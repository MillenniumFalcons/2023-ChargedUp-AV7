package team3647.frc2023.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.Map;
import java.util.function.BooleanSupplier;
import team3647.frc2023.auto.AutoCommands;
import team3647.frc2023.constants.AutoConstants;
import team3647.frc2023.constants.ExtenderConstants;
import team3647.frc2023.constants.FieldConstants;
import team3647.frc2023.constants.GlobalConstants;
import team3647.frc2023.constants.GrabberConstants;
import team3647.frc2023.constants.LimelightConstant;
import team3647.frc2023.constants.PivotConstants;
import team3647.frc2023.constants.SwerveDriveConstants;
import team3647.frc2023.robot.PositionFinder.Level;
import team3647.frc2023.subsystems.Extender;
import team3647.frc2023.subsystems.Grabber;
import team3647.frc2023.subsystems.Pivot;
import team3647.frc2023.subsystems.Superstructure;
import team3647.frc2023.subsystems.Superstructure.StationType;
import team3647.frc2023.subsystems.SwerveDrive;
import team3647.frc2023.subsystems.VisionController;
import team3647.frc2023.subsystems.VisionController.CAMERA_NAME;
import team3647.frc2023.util.AutoSteer;
import team3647.frc2023.util.SuperstructureState;
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
        scheduler.registerSubsystem(swerve, printer, pivot, extender, grabber, visionController);

        configureDefaultCommands();
        configureButtonBindings();
        configureSmartDashboardLogging();
        pivot.setEncoder(PivotConstants.kInitialAngle);
        extender.setEncoder(ExtenderConstants.kMinimumPositionTicks);
        // swerve.setRobotPose(new Pose2d(1.84, 0.42, Rotation2d.fromDegrees(0)));
        swerve.setRobotPose(AutoConstants.kBlueJustScore);
    }

    private void configureButtonBindings() {

        mainController
                .rightTrigger
                .onTrue(
                        Commands.runOnce(
                                () ->
                                        autoSteer.initializeSteering(
                                                positionFinder.getScoringPosition().pose)))
                .whileTrue(
                        new ConditionalCommand(
                                Commands.waitUntil(new Trigger(autoSteer::almostArrived))
                                        .andThen(superstructure.armAutomatic()),
                                superstructure.armCone(),
                                enableAutoSteer));

        mainController.rightStickMoved.onTrue(
                Commands.waitUntil(mainController.rightStickMoved.negate())
                        .andThen(
                                () ->
                                        autoSteer.lockHeading(
                                                Units.degreesToRadians(swerve.getHeading()))));
        mainController.leftBumper.onTrue(superstructure.scoreStowHalfSecDelay());

        mainController
                .rightBumper
                .whileTrue(
                        Commands.runOnce(
                                        () -> {
                                            var intakePos =
                                                    positionFinder.getIntakePositionByStation(
                                                            superstructure.getWantedStation());
                                            if (intakePos.pose == FieldConstants.kGroundIntake) {
                                                return;
                                            }
                                            autoSteer.initializeSteering(intakePos.pose);
                                        })
                                .andThen(Commands.waitUntil(intakeModeChanged))
                                .repeatedly())
                .onTrue(superstructure.intakeAutomatic())
                .onFalse(superstructure.stowFromIntake());

        coController.buttonA.onTrue(superstructure.setWantedLevelCommand(Level.One));
        coController.buttonB.onTrue(superstructure.setWantedLevelCommand(Level.Two));
        coController.buttonY.onTrue(superstructure.setWantedLevelCommand(Level.Three));
        coController.buttonX.onTrue(superstructure.stow());

        coController.dPadDown.onTrue(superstructure.setWantedStationCommand(StationType.Ground));
        coController.dPadUp.onTrue(superstructure.setWantedStationCommand(StationType.Double));
        coController
                .dPadRight
                .or(coController.dPadLeft)
                .onTrue(superstructure.setWantedStationCommand(StationType.Single));

        coController
                .rightBumper
                .whileTrue(
                        superstructure.pivotCommands.openLoopConstant(coController::getRightStickY))
                .whileTrue(
                        superstructure.extenderCommands.openLoopSlow(coController::getLeftStickY));
        coController.rightMidButton.onTrue(superstructure.enableAutoSteer());
        coController.leftMidButton.onTrue(superstructure.disableAutoSteer());
    }

    private void configureDefaultCommands() {
        swerve.setDefaultCommand(
                superstructure.drivetrainCommands.driveVisionTeleop(
                        mainController::getLeftStickX,
                        mainController::getLeftStickY,
                        mainController::getRightStickX,
                        mainController.leftTrigger,
                        // enable autosteer if going to actual station (bumper), or scoring
                        // (trigger)
                        enableAutoSteer,
                        () -> true,
                        AllianceFlipUtil::shouldFlip,
                        autoSteer::findVelocities));
        pivot.setDefaultCommand(superstructure.pivotCommands.holdPositionAtCall());

        grabber.setDefaultCommand(superstructure.grabberCommands.closeGrabber());
        extender.setDefaultCommand(superstructure.extenderCommands.holdPositionAtCall());
    }

    void configTestCommands() {
        Commands.run(() -> {}, extender).schedule();
        Commands.run(() -> {}, pivot).schedule();
        Commands.run(() -> {}, grabber).schedule();
    }

    public double getPivotFFVoltage() {
        return PivotConstants.kG
                * (extender.getNativeTicks() - ExtenderConstants.kMinimumPositionTicks)
                / ExtenderConstants.kMaximumPositionTicks;
    }

    public void configureSmartDashboardLogging() {
        printer.addPose("odo", swerve::getOdoPose);
        printer.addPose("estim", swerve::getEstimPose);
        printer.addPose("Target", () -> positionFinder.getScoringPosition().pose);
        // printer.addPose("vision average", visionController::getAveragedPose);
        printer.addBoolean("autosteer", () -> enableAutoSteer.getAsBoolean());
        printer.addBoolean("auto steer almost ready", () -> autoSteer.almostArrived());
    }

    public Command getAutonomousCommand() {
        return autoCommands.blue.justScore(SuperstructureState.coneThreeReversed);
    }

    private final Joysticks mainController = new Joysticks(0);
    private final Joysticks coController = new Joysticks(1);

    private final Trigger intakeModeChanged =
            coController
                    .dPadLeft
                    .or(coController.dPadRight)
                    .or(coController.dPadUp)
                    .or(coController.dPadDown);

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
                    Map.of(
                            CAMERA_NAME.CENTER,
                            new Limelight(
                                    LimelightConstant.kLimelightCenterIP,
                                    LimelightConstant.kLimelightCenterHost,
                                    0,
                                    LimelightConstant.kCamConstatnts),
                            CAMERA_NAME.LEFT,
                            new Limelight(
                                    LimelightConstant.kLimelightLeftIP,
                                    LimelightConstant.kLimelightLeftHost,
                                    0,
                                    LimelightConstant.kCamConstatnts),
                            CAMERA_NAME.RIGHT,
                            new Limelight(
                                    LimelightConstant.kLimelightRightIP,
                                    LimelightConstant.kLimelightRightHost,
                                    0,
                                    LimelightConstant.kCamConstatnts)),
                    swerve::addVisionMeasurement);

    private final Compressor compressor = new Compressor(GlobalConstants.kPCMType);
    private final PositionFinder positionFinder =
            new PositionFinder(
                    swerve::getEstimPose,
                    FieldConstants.kScoringPositions,
                    FieldConstants.kIntakePositions,
                    SuperstructureState.kLevelPieceMap);
    private final AutoSteer autoSteer =
            new AutoSteer(
                    swerve::getEstimPose,
                    SwerveDriveConstants.kAutoSteerXPIDController,
                    SwerveDriveConstants.kAutoSteerYPIDController,
                    SwerveDriveConstants.kAutoSteerHeadingController);

    private final PowerDistribution pdh = new PowerDistribution(1, ModuleType.kRev);
    final Superstructure superstructure =
            new Superstructure(
                    swerve,
                    pivot,
                    extender,
                    grabber,
                    positionFinder,
                    compressor,
                    intakeModeChanged);
    private final CommandScheduler scheduler = CommandScheduler.getInstance();
    final GroupPrinter printer = GroupPrinter.getInstance();
    private final AutoCommands autoCommands =
            new AutoCommands(swerve, SwerveDriveConstants.kDriveKinematics, superstructure);

    private final Trigger globalEnableAutosteer = new Trigger(superstructure::autoSteerEnabled);

    private final BooleanSupplier enableAutoSteer =
            globalEnableAutosteer.and(
                    mainController.rightTrigger.or(
                            mainController.rightBumper.and(
                                    () ->
                                            superstructure.getWantedStation()
                                                    != StationType.Ground)));
}
