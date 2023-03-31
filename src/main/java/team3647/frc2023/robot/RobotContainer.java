package team3647.frc2023.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.Map;
import java.util.function.BooleanSupplier;
import team3647.frc2023.auto.AutoCommands;
import team3647.frc2023.auto.AutonomousMode;
import team3647.frc2023.constants.ExtenderConstants;
import team3647.frc2023.constants.FieldConstants;
import team3647.frc2023.constants.GlobalConstants;
import team3647.frc2023.constants.LimelightConstant;
import team3647.frc2023.constants.PivotConstants;
import team3647.frc2023.constants.RollersConstants;
import team3647.frc2023.constants.SwerveDriveConstants;
import team3647.frc2023.constants.WristConstants;
import team3647.frc2023.robot.PositionFinder.GamePiece;
import team3647.frc2023.robot.PositionFinder.Level;
import team3647.frc2023.robot.PositionFinder.ScoringPosition;
import team3647.frc2023.robot.PositionFinder.Side;
import team3647.frc2023.subsystems.Extender;
import team3647.frc2023.subsystems.Pivot;
import team3647.frc2023.subsystems.Rollers;
import team3647.frc2023.subsystems.Superstructure;
import team3647.frc2023.subsystems.Superstructure.StationType;
import team3647.frc2023.subsystems.SwerveDrive;
import team3647.frc2023.subsystems.VisionController;
import team3647.frc2023.subsystems.VisionController.CAMERA_NAME;
import team3647.frc2023.subsystems.Wrist;
import team3647.frc2023.util.AutoSteer;
import team3647.frc2023.util.SuperstructureState;
import team3647.lib.GroupPrinter;
import team3647.lib.inputs.Joysticks;
import team3647.lib.tracking.FlightDeck;
import team3647.lib.tracking.RobotTracker;
import team3647.lib.vision.AimingParameters;
import team3647.lib.vision.Limelight;
import team3647.lib.vision.MultiTargetTracker;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    private AutonomousMode runningMode;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        pdh.clearStickyFaults();
        scheduler.registerSubsystem(
                swerve, printer, pivot, extender, visionController, rollers, wrist);

        configureDefaultCommands();
        configureButtonBindings();
        configureSmartDashboardLogging();
        pivot.setEncoder(PivotConstants.kInitialAngle);
        extender.setEncoder(ExtenderConstants.kMinimumPositionTicks);
        wrist.setEncoder(WristConstants.kInitialDegree);
        runningMode = autoCommands.blueConeCubeBalanceFlatSideMode;

        swerve.setRobotPose(runningMode.getPathplannerPose2d());
    }

    private void configureButtonBindings() {

        mainController
                .rightTrigger
                .and(goodForAutosteer)
                .onTrue(
                        Commands.runOnce(
                                () ->
                                        autoSteer.initializeSteering(
                                                superstructure.getScoringPosition().pose)))
                .whileTrue(superstructure.armAutomatic());
        mainController.rightTrigger.onFalse(superstructure.scoreStowNoDelay());
        new Trigger(mainController::anyStickMovedStiff).onTrue(Commands.runOnce(autoSteer::stop));
        mainController.buttonA.whileTrue(superstructure.intakeForCurrentGamePiece());
        mainController
                .rightTrigger
                .and(() -> !goodForAutosteer.getAsBoolean())
                .onTrue(superstructure.armToPieceFromSide());

        mainController.rightBumper.whileTrue(superstructure.intakeAutomatic());

        mainController.dPadUp.onTrue(superstructure.higherWristOffset());
        mainController.dPadDown.onTrue(superstructure.lowerWristOffset());

        coController.buttonA.onTrue(superstructure.setWantedLevelCommand(Level.Ground));
        coController.buttonB.onTrue(superstructure.setWantedLevelCommand(Level.Two));
        coController.buttonY.onTrue(superstructure.setWantedLevelCommand(Level.Three));
        coController.buttonX.onTrue(superstructure.stowAll());

        coController.dPadLeft.onTrue(superstructure.setWantedSideCommand(Side.Left));
        coController.dPadRight.onTrue(superstructure.setWantedSideCommand(Side.Right));
        coController.dPadUp.onTrue(superstructure.setWantedSideCommand(Side.Center));

        coController
                .rightBumper
                .or(coController.leftBumper)
                .onTrue(superstructure.setWantedStationCommand(StationType.Double));
        coController
                .rightTrigger
                .or(coController.leftTrigger)
                .onTrue(superstructure.setWantedStationCommand(StationType.Ground));

        coController
                .leftBumper
                .or(coController.leftTrigger)
                .onTrue(superstructure.setWantedIntakeGamePieceCommand(GamePiece.Cone));

        coController
                .rightBumper
                .or(coController.rightTrigger)
                .onTrue(superstructure.setWantedIntakeGamePieceCommand(GamePiece.Cube));

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
                        goodForAutosteer,
                        () -> true,
                        autoSteer::findVelocities));

        // wrist.setDefaultCommand(superstructure.wristCommands.holdPositionAtCall());
        pivot.setDefaultCommand(superstructure.pivotCommands.holdPositionAtCall());
        extender.setDefaultCommand(superstructure.extenderCommands.holdPositionAtCall());
    }

    public void teleopInit() {
        rollers.setDefaultCommand(superstructure.holdForCurrentGamePiece());
    }

    void configTestCommands() {
        Commands.run(() -> {}, extender).schedule();
        Commands.run(() -> {}, pivot).schedule();
        // Commands.run(() -> {}, grabber).schedule();
    }

    public double getPivotFFVoltage() {
        return PivotConstants.kG
                * (extender.getNativeTicks() - ExtenderConstants.kMinimumPositionTicks)
                / ExtenderConstants.kMaximumPositionTicks;
    }

    public void configureSmartDashboardLogging() {
        printer.addPose("odo", swerve::getOdoPose);
        printer.addDouble("Extender amps", extender::getMasterCurrent);
        printer.addDouble("PIVOT", pivot::getAngle);
        printer.addDouble("Wrist", wrist::getAngle);
        printer.addDouble("extender", extender::getNativePos);
        printer.addBoolean("autosteer", goodForAutosteer::getAsBoolean);
        printer.addBoolean("auto steer almost ready", autoSteer::almostArrived);
        printer.addPose("Cam pose", flightDeck::getFieldToCamera);
        printer.addPose("AutoSteerTarget", () -> superstructure.getScoringPosition().pose);
        printer.addBoolean("Valid Target", superstructure::validScoringPosition);
        printer.addPose("Cube score", () -> getPoseIfLength(Side.Center.listIndex).getPose());
        printer.addPose("Cone Left", () -> getPoseIfLength(Side.Left.listIndex).getPose());
        printer.addPose("Cone Right", () -> getPoseIfLength(Side.Right.listIndex).getPose());

        printer.addPose(
                "April Pose",
                () -> {
                    var params = flightDeck.getLatestParameters();
                    if (params == AimingParameters.None) {
                        return kEmptyPose;
                    }
                    return params.getFieldToGoal();
                });
        printer.addBoolean(
                "Cube Ground",
                () ->
                        superstructure.getWantedLevel() == Level.Ground
                                && superstructure.getWantedStation() == StationType.Ground);
    }

    private ScoringPosition getPoseIfLength(int idx) {
        if (superstructure.getAllScoringPositions().size() != 3) {
            return ScoringPosition.kNone;
        }
        return superstructure.getAllScoringPositions().get(idx);
    }

    // counted relative to what driver sees
    public Command getAutonomousCommand() {
        return runningMode.getAutoCommand();
    }

    private final Joysticks mainController = new Joysticks(0);
    private final Joysticks coController = new Joysticks(1);

    private final Trigger wantedSideChanged =
            coController
                    .dPadLeft
                    .or(coController.dPadRight)
                    .or(coController.dPadUp)
                    .or(coController.dPadDown);

    private final Trigger intakeChanged = coController.leftBumper.or(coController.rightBumper);

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

    public final Rollers rollers =
            new Rollers(
                    RollersConstants.kMaster,
                    RollersConstants.kCubeSensor,
                    1,
                    1,
                    RollersConstants.kNominalVoltage,
                    GlobalConstants.kDt);
    public final Wrist wrist =
            new Wrist(
                    WristConstants.kMaster,
                    WristConstants.kNativeVelToDPS,
                    WristConstants.kNativePosToDegrees,
                    WristConstants.kMinDegree,
                    WristConstants.kMaxDegree,
                    WristConstants.nominalVoltage,
                    WristConstants.kG,
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
                    ExtenderConstants.kMinimumPositionTicks,
                    ExtenderConstants.kMaximumPositionTicks,
                    ExtenderConstants.nominalVoltage,
                    GlobalConstants.kDt);

    final FlightDeck flightDeck =
            new FlightDeck(
                    new RobotTracker(1.0, swerve::getOdoPose, swerve::getTimestamp),
                    new MultiTargetTracker(),
                    LimelightConstant.kRobotToCamFixed);

    private final VisionController visionController =
            new VisionController(
                    Map.of(
                            CAMERA_NAME.CENTER,
                            new Limelight(
                                    LimelightConstant.kLimelightCenterIP,
                                    LimelightConstant.kLimelightCenterHost,
                                    0,
                                    LimelightConstant.kCamConstants)),
                    flightDeck::addVisionObservation,
                    FieldConstants.kScoreTargetHeightMeters,
                    FieldConstants.kIntakeTargetHeightMeters);

    private final Compressor compressor = new Compressor(GlobalConstants.kPCMType);
    private final PositionFinder positionFinder =
            new PositionFinder(
                    swerve::getOdoPose,
                    flightDeck::getLatestParameters,
                    SuperstructureState.kLevelPieceMap);

    private final AutoSteer autoSteer =
            new AutoSteer(
                    swerve::getOdoPose,
                    SwerveDriveConstants.kAutoSteerXPIDController,
                    SwerveDriveConstants.kAutoSteerYPIDController,
                    SwerveDriveConstants.kAutoSteerHeadingController);

    private final PowerDistribution pdh = new PowerDistribution(1, ModuleType.kRev);
    final Superstructure superstructure =
            new Superstructure(
                    swerve,
                    pivot,
                    extender,
                    rollers,
                    wrist,
                    positionFinder,
                    compressor,
                    mainController::anyStickMovedFast);
    private final CommandScheduler scheduler = CommandScheduler.getInstance();
    final GroupPrinter printer = GroupPrinter.getInstance();
    private final AutoCommands autoCommands =
            new AutoCommands(swerve, SwerveDriveConstants.kDriveKinematics, superstructure);

    private final Command justScoreLevel3Cone =
            autoCommands.justScore(SuperstructureState.coneThreeReversed);
    private final Command justScoreLevel3Cube =
            autoCommands.justScore(SuperstructureState.cubeThreeReversed);

    private final Trigger globalEnableAutosteer = new Trigger(superstructure::autoSteerEnabled);

    private final BooleanSupplier goodForAutosteer =
            globalEnableAutosteer
                    .and(mainController.rightTrigger)
                    .and(superstructure::validScoringPosition);
    private final Pose2d kEmptyPose = new Pose2d();
}
