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
import java.util.function.BooleanSupplier;
import team3647.frc2023.auto.AutoCommands;
import team3647.frc2023.auto.AutonomousMode;
import team3647.frc2023.constants.CubeShooterConstants;
import team3647.frc2023.constants.CubeWristConstants;
import team3647.frc2023.constants.ExtenderConstants;
import team3647.frc2023.constants.GlobalConstants;
import team3647.frc2023.constants.LEDConstants;
import team3647.frc2023.constants.LimelightConstant;
import team3647.frc2023.constants.PivotConstants;
import team3647.frc2023.constants.RollersConstants;
import team3647.frc2023.constants.SwerveDriveConstants;
import team3647.frc2023.constants.WristConstants;
import team3647.frc2023.robot.PositionFinder.GamePiece;
import team3647.frc2023.robot.PositionFinder.Level;
import team3647.frc2023.subsystems.CubeShooterBottom;
import team3647.frc2023.subsystems.CubeShooterTop;
import team3647.frc2023.subsystems.CubeWrist;
import team3647.frc2023.subsystems.Extender;
import team3647.frc2023.subsystems.LEDSubsystem;
import team3647.frc2023.subsystems.LEDSubsystem.LEDStates;
import team3647.frc2023.subsystems.Pivot;
import team3647.frc2023.subsystems.Rollers;
import team3647.frc2023.subsystems.Superstructure;
import team3647.frc2023.subsystems.Superstructure.StationType;
import team3647.frc2023.subsystems.SwerveDrive;
import team3647.frc2023.subsystems.Wrist;
import team3647.frc2023.util.AutoSteer;
import team3647.frc2023.util.SuperstructureState;
import team3647.lib.GroupPrinter;
import team3647.lib.inputs.Joysticks;
import team3647.lib.tracking.FlightDeck;
import team3647.lib.tracking.RobotTracker;
import team3647.lib.vision.LimelightHelpers;
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
                swerve,
                printer,
                pivot,
                extender,
                rollers,
                wrist,
                cubeWrist,
                cubeShooterBottom,
                cubeShooterTop,
                LEDS);

        configureDefaultCommands();
        configureButtonBindings();
        configureSmartDashboardLogging();
        pivot.setEncoder(PivotConstants.kInitialAngle);
        extender.setEncoder(ExtenderConstants.kMinimumPositionTicks);
        wrist.setEncoder(WristConstants.kInitialDegree);
        cubeWrist.setEncoder(CubeWristConstants.kInitialDegree);
        runningMode = autoCommands.redConeCubeBalanceFlatSideMode;
        LimelightHelpers.setPipelineIndex(LimelightConstant.kLimelightCenterHost, 1);

        swerve.setRobotPose(runningMode.getPathplannerPose2d());
    }

    private void configureButtonBindings() {
        // need to change this to a conditional command so it doesn't start auto aiming
        // when doing
        // cubes from cube shooter
        mainController
                .rightTrigger
                .and(() -> !superstructure.isBottomF())
                .whileTrue(superstructure.armAutomatic())
                .onTrue(Commands.runOnce(autoSteer::initializeSteering))
                .onTrue(
                        Commands.runOnce(
                                () ->
                                        LimelightHelpers.setLEDMode_ForceOn(
                                                LimelightConstant.kLimelightCenterHost)));
        // LED
        mainController.rightTrigger.onTrue(
                Commands.runOnce(() -> LEDS.setLEDState(LEDStates.TARGET)));

        mainController
                .rightTrigger
                .and(() -> !superstructure.isBottomF())
                .and(mainController.leftBumper.negate())
                .onFalse(
                        Commands.runOnce(
                                () ->
                                        LimelightHelpers.setLEDMode_ForceOff(
                                                LimelightConstant.kLimelightCenterHost)))
                .onFalse(superstructure.scoreAndStowLonger(0.4).unless(mainController.buttonB))
                .onFalse(Commands.runOnce(autoSteer::stop));
        // LED
        mainController.rightTrigger.onFalse(
                Commands.runOnce(
                        () -> {
                            LEDS.setPieceIn(false);
                            LEDS.setLEDState(LEDStates.IDLE);
                        }));

        mainController
                .rightTrigger
                .and(superstructure::isBottomF)
                .onTrue(superstructure.shootAutomatic());

        mainController.buttonA.whileTrue(superstructure.intakeForCurrentGamePiece());

        mainController.rightBumper.whileTrue(superstructure.intakeAutomatic());
        mainController
                .rightBumper
                .and(() -> superstructure.getWantedStation() == StationType.Double)
                .onTrue(Commands.runOnce(() -> autoSteer.justHeading(0)));
        // LED
        mainController.rightBumper.onFalse(Commands.runOnce(() -> LEDS.setPieceIn(true)));

        mainController.leftBumper.whileTrue(superstructure.cubeShooterIntake());
        mainController.leftBumper.onFalse(superstructure.cubeShooterCommands.stow());
        // LED
        mainController.leftBumper.onFalse(Commands.runOnce(() -> LEDS.setPieceIn(true)));

        mainController.dPadUp.onTrue(superstructure.higherWristOffset());
        mainController.dPadDown.onTrue(superstructure.lowerWristOffset());

        coController.buttonA.onTrue(superstructure.setWantedLevelCommand(Level.Ground));
        coController.buttonB.onTrue(superstructure.setWantedLevelCommand(Level.Two));
        coController.buttonY.onTrue(superstructure.setWantedLevelCommand(Level.Three));
        coController.buttonX.onTrue(superstructure.stowAll());

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

        // LED Triggers
        currentCone.onTrue(Commands.runOnce(() -> LEDS.setPiece(true)));
        currentCube.onTrue(Commands.runOnce(() -> LEDS.setPiece(false)));

        wantedCone.onTrue(Commands.runOnce(() -> LEDS.setWantedPiece(true)));
        wantedCube.onTrue(Commands.runOnce(() -> LEDS.setWantedPiece(false)));

        bottomCubeIn.onTrue(
                Commands.runOnce(() -> LEDS.setBottomCubeIn(true))
                        .withTimeout(2)
                        .andThen(() -> LEDS.setBottomCubeIn(false)));
        bottomCubeIn.onFalse(Commands.runOnce(() -> LEDS.setBottomCubeIn(false)));

        seesTarget.onTrue(Commands.runOnce(() -> LEDS.setTarget(true)));
        seesTarget.onFalse(Commands.runOnce(() -> LEDS.setTarget(false)));

        coControllerRightJoystickMoved.onTrue(
                Commands.runOnce(
                        () -> {
                            LEDS.setLEDState(LEDStates.RAINBOW);
                        }));
        coControllerRightJoystickMoved.whileTrue(
                Commands.runOnce(
                        () -> {
                            LEDConstants.RAINBOWCONTROLLER.setSpeed(
                                    (2 - Math.abs(coController.getRightStickY())));
                        }));
        coControllerRightJoystickMoved.onFalse(
                Commands.runOnce(
                        () -> {
                            LEDS.setLEDState(LEDStates.IDLE);
                        }));
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
        cubeWrist.setDefaultCommand(superstructure.cubeShooterCommands.holdPositionAtCall());
        cubeShooterTop.setDefaultCommand(superstructure.cubeShooterTopDefault());
        cubeShooterBottom.setDefaultCommand(superstructure.cubeShooterBottomDefault());
    }

    public void teleopInit() {
        rollers.setDefaultCommand(superstructure.holdForCurrentGamePiece());
        if (superstructure.getWantedIntakePiece() == GamePiece.Cone) {
            LEDS.setPiece(true);
        } else if (superstructure.getWantedIntakePiece() == GamePiece.Cube) {
            LEDS.setPiece(false);
        }
        LEDS.setLEDState(LEDStates.IDLE);
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

        printer.addBoolean("ground intake", superstructure::isBottomF);
        printer.addString("LED State", LEDS::getLEDState);
        printer.addBoolean("pieceIn", LEDS::getPieceIn);
        printer.addBoolean("tof Dist", cubeWrist::isSensorTriggered);

        printer.addBoolean(
                "Cube Ground",
                () ->
                        superstructure.getWantedLevel() == Level.Ground
                                && superstructure.getWantedStation() == StationType.Ground);
    }

    // counted relative to what driver sees
    public Command getAutonomousCommand() {
        return runningMode.getAutoCommand();
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
                    SwerveDriveConstants.kRotPossibleMaxSpeedRadPerSec,
                    GlobalConstants.kDt);

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

    public final CubeWrist cubeWrist =
            new CubeWrist(
                    CubeWristConstants.kMaster,
                    CubeWristConstants.timeOfFlight,
                    CubeWristConstants.kNativeVelToDPS,
                    CubeWristConstants.kNativePosToDegrees,
                    CubeWristConstants.nominalVoltage,
                    CubeWristConstants.kG,
                    CubeWristConstants.kMinDegree,
                    CubeWristConstants.kMaxDegree,
                    GlobalConstants.kDt);

    public final CubeShooterTop cubeShooterTop =
            new CubeShooterTop(
                    CubeShooterConstants.kTopRoller,
                    1,
                    1,
                    CubeShooterConstants.kNominalVoltage,
                    GlobalConstants.kDt);

    public final CubeShooterBottom cubeShooterBottom =
            new CubeShooterBottom(
                    CubeShooterConstants.kBottomRoller,
                    1,
                    1,
                    CubeShooterConstants.kNominalVoltage,
                    GlobalConstants.kDt);

    public final LEDSubsystem LEDS = new LEDSubsystem();

    final FlightDeck flightDeck =
            new FlightDeck(
                    new RobotTracker(1.0, swerve::getOdoPose, swerve::getTimestamp),
                    new MultiTargetTracker(),
                    LimelightConstant.kRobotToCamFixed);

    private final Compressor compressor = new Compressor(GlobalConstants.kPCMType);
    private final PositionFinder positionFinder =
            new PositionFinder(
                    swerve::getOdoPose,
                    flightDeck::getLatestParameters,
                    SuperstructureState.kLevelPieceMap);

    private final AutoSteer autoSteer =
            new AutoSteer(
                    swerve::getOdoPose,
                    () -> -LimelightHelpers.getTX(LimelightConstant.kLimelightCenterHost),
                    SwerveDriveConstants.kAutoSteerXYPIDController,
                    SwerveDriveConstants.kAutoSteerXYPIDController,
                    SwerveDriveConstants.kAutoSteerHeadingController);

    private final PowerDistribution pdh = new PowerDistribution(1, ModuleType.kRev);
    public final Superstructure superstructure =
            new Superstructure(
                    swerve,
                    pivot,
                    extender,
                    rollers,
                    wrist,
                    cubeShooterTop,
                    cubeShooterBottom,
                    cubeWrist,
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
                    .and(() -> !superstructure.isBottomF())
                    .and(() -> superstructure.getWantedLevel() != Level.Ground)
                    .and(() -> superstructure.getGamePiece() == GamePiece.Cone)
                    .or(
                            mainController.rightBumper.and(
                                    () -> superstructure.getWantedStation() == StationType.Double));

    private final Pose2d kEmptyPose = new Pose2d();

    // LED Triggers
    private final Trigger currentCone =
            new Trigger(() -> superstructure.getCurrentIntakePiece() == GamePiece.Cone);
    private final Trigger currentCube =
            new Trigger(
                    () ->
                            superstructure.getCurrentIntakePiece() == GamePiece.Cube
                                    || superstructure.isBottomF());
    private final Trigger wantedCone =
            new Trigger(() -> superstructure.getWantedIntakePiece() == GamePiece.Cone);
    private final Trigger wantedCube =
            new Trigger(
                    () ->
                            superstructure.getWantedIntakePiece() == GamePiece.Cube
                                    && !superstructure.isBottomF());

    private final Trigger seesTarget = new Trigger(autoSteer::almostArrived);

    private final Trigger bottomCubeIn = new Trigger(cubeWrist::isSensorTriggered);

    private final Trigger coControllerRightJoystickMoved =
            new Trigger(() -> Math.abs(coController.getRightStickY()) >= 0.2);
}
