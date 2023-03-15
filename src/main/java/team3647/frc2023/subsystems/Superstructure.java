package team3647.frc2023.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import team3647.frc2023.commands.DrivetrainCommands;
import team3647.frc2023.commands.ExtenderCommands;
import team3647.frc2023.commands.PivotCommands;
import team3647.frc2023.commands.RollersCommands;
import team3647.frc2023.commands.WristCommands;
import team3647.frc2023.robot.PositionFinder;
import team3647.frc2023.robot.PositionFinder.GamePiece;
import team3647.frc2023.robot.PositionFinder.Level;
import team3647.frc2023.robot.PositionFinder.ScoringPosition;
import team3647.frc2023.robot.PositionFinder.Side;
import team3647.frc2023.util.SuperstructureState;
import team3647.lib.GroupPrinter;

public class Superstructure {

    private Level wantedLevel = Level.Stay;
    private StationType wantedStation = StationType.Double;
    private Side wantedSide = Side.Center;
    private boolean isAutoSteerEnabled = true;
    private List<ScoringPosition> scoringPositions =
            List.of(ScoringPosition.kNone, ScoringPosition.kNone, ScoringPosition.kNone);
    private ScoringPosition scoringPositionBySide = ScoringPosition.kNone;
    private GamePiece gamePieceForManual = GamePiece.Cone;
    private SuperstructureState wantedIntakeState = SuperstructureState.doubleStation;
    private GamePiece intakeGamePiece = GamePiece.Cone;

    private final Translation2d kMoveIntoField = new Translation2d(0.4, 0);

    public void periodic(double timestamp) {
        scoringPositions = finder.getScoringPositions();
        scoringPositionBySide = finder.getPositionBySide(getWantedSide());
        gamePieceForManual =
                getWantedSide() == Side.Center || getWantedLevel() == Level.Ground
                        ? GamePiece.Cube
                        : GamePiece.Cone;
        wantedIntakeState =
                getWantedStation() == StationType.Ground
                        ? SuperstructureState.groundIntake
                        : SuperstructureState.doubleStation;

        var defaultPiece = !switchPiece.getAsBoolean();
        boolean groundIntake = getWantedStation() == StationType.Ground;
        if (groundIntake == defaultPiece) {
            intakeGamePiece = GamePiece.Cube;
        } else {
            intakeGamePiece = GamePiece.Cone;
        }

        if (getWantedLevel() == Level.Ground) {
            // shift pose into the field so we don't kill the arm +x
            scoringPositionBySide =
                    new ScoringPosition(
                            new Pose2d(
                                    scoringPositionBySide
                                            .pose
                                            .getTranslation()
                                            .plus(kMoveIntoField),
                                    scoringPositionBySide.pose.getRotation()),
                            GamePiece.Cube);
        }
        SmartDashboard.putString(
                "Game Piece", scoringPositionBySide.piece == GamePiece.Cone ? "CONE" : "CUBE");
    }

    public Command scoreStowHalfSecDelay() {
        return scoreAndStow(0.5);
    }

    public Command scoreStowNoDelay() {
        return scoreAndStow(0);
    }

    public Command armAutomatic() {
        return goToStateParallel(
                () ->
                        finder.getSuperstructureStateByPiece(
                                getWantedLevel(), getScoringPosition().piece));
    }

    public Command intakeAutomatic() {
        return Commands.deadline(
                        waitForCurrentSpike(),
                        Commands.parallel(
                                goToStateParallel(() -> this.wantedIntakeState),
                                intakeForGamePiece(() -> this.intakeGamePiece)))
                .finallyDo(interrupted -> stowFromIntake().schedule());
    }

    public Command intakeForGamePiece(Supplier<GamePiece> piece) {
        return Commands.run(
                () -> {
                    if (piece.get() == GamePiece.Cone) {
                        rollers.intakeCone();
                    } else {
                        rollers.intakeCube();
                    }
                },
                rollers);
    }

    public Command holdForGamePiece(Supplier<GamePiece> piece) {
        return Commands.run(
                () -> {
                    if (piece.get() == GamePiece.Cone) {
                        rollers.setOpenloop(-0.05);
                    } else {
                        rollers.setOpenloop(0.01);
                    }
                },
                rollers);
    }

    public Command waitForCurrentSpike() {
        return Commands.sequence(
                Commands.waitSeconds(1), Commands.waitUntil(() -> rollers.getMasterCurrent() > 20));
    }

    public Command stowFromIntake() {
        return Commands.deadline(stowIntake(), holdForGamePiece(() -> this.intakeGamePiece));
    }

    public Command armToPieceFromSide() {
        return goToStateParallel(
                () -> finder.getSuperstructureStateByPiece(getWantedLevel(), gamePieceForManual));
    }

    public Command goToStateParallel(SuperstructureState state) {
        return Commands.run(() -> runPivotExtenderWrist(state), pivot, extender, wrist)
                .until(() -> pivotExtenderReached(state));
    }

    public Command goToStateParallel(Supplier<SuperstructureState> getState) {
        return Commands.run(() -> runPivotExtenderWrist(getState.get()), pivot, extender, wrist);
    }

    private final double kMaxRotationLength = 15000;

    private void runPivotExtenderWrist(SuperstructureState wantedState) {
        boolean currentBelowMaxRotateLength = extender.getNativePos() < kMaxRotationLength + 500;
        boolean nextBelowMaxRotateLength = wantedState.length < kMaxRotationLength + 500;
        boolean needsRotate = !pivot.angleReached(wantedState.armAngle, 5);
        boolean needsExtend = !extender.reachedPosition(wantedState.length, 5000);
        boolean closeEnoughForParallel = pivot.angleReached(wantedState.armAngle, 8);
        double nextExtender =
                currentBelowMaxRotateLength ? extender.getNativePos() : kMaxRotationLength;
        double nextPivot = pivot.getAngle();
        if (needsRotate && !closeEnoughForParallel) {
            if (currentBelowMaxRotateLength && nextBelowMaxRotateLength) {
                nextExtender = wantedState.length;
                nextPivot = wantedState.armAngle;
            } else if (currentBelowMaxRotateLength) {
                nextPivot = wantedState.armAngle;
            }
        } else if (closeEnoughForParallel && !needsExtend) {
            nextPivot = wantedState.armAngle;
            nextExtender = wantedState.length;
        } else {
            nextExtender = wantedState.length;
        }

        SmartDashboard.putNumber("Wanted Wrist", wantedState.wristAngle);
        SmartDashboard.putNumber("Wanted extender", nextExtender);
        SmartDashboard.putNumber("Wanted pivot", nextPivot);
        pivot.setAngle(nextPivot);
        extender.setLengthMeters(nextExtender);
        wrist.setAngle(wantedState.wristAngle);
    }

    public boolean pivotExtenderReached(SuperstructureState state) {
        return extender.reachedPosition(state.length, 2000)
                && pivot.angleReached(state.armAngle, 1.5);
    }

    public Command scoreAndStow(double secsBetweenOpenAndStow) {
        return Commands.sequence(
                new ConditionalCommand(
                                score(getScoringPosition().piece),
                                score(gamePieceForManual),
                                () -> this.isAutoSteerEnabled)
                        .withTimeout(0.5),
                Commands.waitSeconds(secsBetweenOpenAndStow),
                stowScore());
    }

    public Command score(GamePiece piece) {
        return new ConditionalCommand(
                rollersCommands.outCone(),
                rollersCommands.outCube(),
                () -> piece == GamePiece.Cone);
    }

    public Command scoreAndStowCube(double secsBetweenOpenAndStow) {
        return Commands.sequence(
                pivotCommands.goDownDegrees(5),
                rollersCommands.outCone().withTimeout(1),
                Commands.waitSeconds(secsBetweenOpenAndStow),
                stowScore());
    }

    public Command doubleStation() {
        return goToStateParallel(SuperstructureState.doubleStation);
    }

    public Command groundIntake() {
        return goToStateParallel(SuperstructureState.groundIntake);
    }

    public Command stow() {
        return goToStateParallel(SuperstructureState.stowAll);
    }

    public Command stowIntake() {
        return goToStateParallel(SuperstructureState.stowIntake);
    }

    public Command stowScore() {
        return goToStateParallel(SuperstructureState.stowScore);
    }

    public Command stowAll() {
        return goToStateParallel(SuperstructureState.stowAll);
    }

    public Command disableCompressor() {
        return new InstantCommand(compressor::disable);
    }

    public Command enableCompressor() {
        return new InstantCommand(compressor::enableDigital);
    }

    public Command setWantedLevelCommand(Level level) {
        return Commands.runOnce(() -> setWantedLevel(level));
    }

    public Command setWantedStationCommand(StationType station) {
        return Commands.runOnce(() -> setWantedStation(station));
    }

    public Command setWantedSideCommand(Side side) {
        return Commands.runOnce(() -> setWantedSide(side));
    }

    public Command enableAutoSteer() {
        return Commands.runOnce(() -> this.isAutoSteerEnabled = true);
    }

    public Command disableAutoSteer() {
        return Commands.runOnce(() -> this.isAutoSteerEnabled = false);
    }

    public StationType getWantedStation() {
        return this.wantedStation;
    }

    public void setWantedStation(StationType station) {
        this.wantedStation = station;
    }

    public Level getWantedLevel() {
        return this.wantedLevel;
    }

    public void setWantedLevel(Level level) {
        this.wantedLevel = level;
    }

    public Side getWantedSide() {
        return this.wantedSide;
    }

    public void setWantedSide(Side side) {
        this.wantedSide = side;
    }

    public boolean autoSteerEnabled() {
        return this.isAutoSteerEnabled;
    }

    public List<ScoringPosition> getAllScoringPositions() {
        return this.scoringPositions;
    }

    public ScoringPosition getScoringPosition() {
        return this.scoringPositionBySide;
    }

    /**
     * @return whether the current auto steer target exists, and if it exists whether it is within 1
     *     meter from the robot
     */
    public boolean validScoringPosition() {
        return this.scoringPositionBySide != ScoringPosition.kNone
                || this.scoringPositionBySide
                                .pose
                                .minus(drive.getOdoPose())
                                .getTranslation()
                                .getNorm()
                        < 2;
    }

    // keep this at the bottom
    public Superstructure(
            SwerveDrive drive,
            Pivot pivot,
            Extender extender,
            Rollers rollers,
            Wrist wrist,
            PositionFinder finder,
            Compressor compressor,
            BooleanSupplier switchPiece,
            BooleanSupplier drivetrainWantMove) {
        this.drive = drive;
        this.pivot = pivot;
        this.extender = extender;
        this.rollers = rollers;
        this.wrist = wrist;
        this.finder = finder;
        this.compressor = compressor;

        drivetrainCommands = new DrivetrainCommands(drive);
        pivotCommands = new PivotCommands(pivot);
        extenderCommands = new ExtenderCommands(extender);
        rollersCommands = new RollersCommands(rollers);
        wristCommands = new WristCommands(wrist);
        this.switchPiece = switchPiece;
        this.drivetrainWantMove = drivetrainWantMove;
    }

    private double kGPivot;
    private final Compressor compressor;
    private final SwerveDrive drive;
    private final Pivot pivot;
    private final Extender extender;
    private final Rollers rollers;
    private final Wrist wrist;
    private final GroupPrinter printer = GroupPrinter.getInstance();
    private final PositionFinder finder;
    private final BooleanSupplier switchPiece;
    public final DrivetrainCommands drivetrainCommands;
    public final PivotCommands pivotCommands;
    public final ExtenderCommands extenderCommands;
    public final RollersCommands rollersCommands;
    public final WristCommands wristCommands;
    private final BooleanSupplier drivetrainWantMove;

    public enum StationType {
        Single,
        Double,
        Ground
    }
}
