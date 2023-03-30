package team3647.frc2023.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
    private SuperstructureState wantedIntakeState = SuperstructureState.doubleStationCone;
    private GamePiece intakeGamePiece = GamePiece.Cone;
    private GamePiece currentGamePiece = GamePiece.Cube;
    private double wristAdjust = 0.0;

    private final Translation2d kMoveIntoField = new Translation2d(0.05, 0);

    public void periodic(double timestamp) {
        scoringPositions = finder.getScoringPositions();
        scoringPositionBySide = finder.getPositionBySide(getWantedSide());
        if (getWantedStation() == StationType.Ground) {
            wantedIntakeState =
                    intakeGamePiece == GamePiece.Cone
                            ? SuperstructureState.groundIntakeCone
                            : SuperstructureState.groundIntakeCube;
        } else {
            wantedIntakeState =
                    intakeGamePiece == GamePiece.Cone
                            ? SuperstructureState.doubleStationCone
                            : SuperstructureState.doubleStationCube;
        }
        if (Math.abs(wristAdjust) > 0.01) {
            wantedIntakeState = wantedIntakeState.addWrist(wristAdjust);
        }

        if (getWantedLevel() == Level.Ground) {
            // shift pose into the field so we don't kill the arm +x
            scoringPositionBySide =
                    new ScoringPosition(scoringPositionBySide.pose, currentGamePiece);
        }
        SmartDashboard.putString(
                "Game Piece", currentGamePiece == GamePiece.Cone ? "CONE" : "CUBE");
        SmartDashboard.putNumber("rollers current", rollers.getMasterCurrent());
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
                .finallyDo(
                        interrupted -> {
                            this.currentGamePiece = this.intakeGamePiece;
                            stowFromIntake().schedule();
                        });
    }

    public Command intakeForCurrentGamePiece() {
        return Commands.startEnd(
                () -> {
                    if (this.currentGamePiece == GamePiece.Cone) {
                        rollers.intakeCone();
                    } else {
                        rollers.intakeCube();
                    }
                },
                rollers::end,
                rollers);
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

    public Command holdForCurrentGamePiece() {
        return holdForGamePiece(() -> this.currentGamePiece);
    }

    public Command holdForGamePiece(Supplier<GamePiece> piece) {
        return Commands.run(
                () -> {
                    if (piece.get() == GamePiece.Cone) {
                        rollers.setOpenloop(-0.1);
                    } else {
                        rollers.setOpenloop(0.03);
                    }
                },
                rollers);
    }

    public Command waitForCurrentSpike() {
        return waitForCurrentSpike(20);
    }

    public Command waitForCurrentSpike(double amps) {
        return Commands.sequence(
                Commands.waitSeconds(1),
                Commands.waitUntil(new Trigger(() -> rollers.getMasterCurrent() > amps)),
                Commands.waitSeconds(0.5));
    }

    public Command waitForHasCube() {
        return Commands.sequence(Commands.waitUntil(rollers::hasCube));
    }

    public Command waitForCurrentSpikeFast(double amps) {
        return Commands.sequence(
                Commands.waitSeconds(0.2),
                Commands.waitUntil(new Trigger(() -> rollers.getMasterCurrent() > amps)));
    }

    public Command stowFromIntake() {
        return Commands.deadline(stowIntake(), holdForCurrentGamePiece());
    }

    public Command armToPieceFromSide() {
        return goToStateParallel(
                () -> finder.getSuperstructureStateByPiece(getWantedLevel(), currentGamePiece));
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
        boolean currentBelowMaxRotateLength = extender.getNativePos() < kMaxRotationLength + 2048;
        boolean nextBelowMaxRotateLength = wantedState.length <= kMaxRotationLength;
        boolean needsRotate = !pivot.angleReached(wantedState.armAngle, 5);
        boolean needsExtend = !extender.reachedPosition(wantedState.length, 5000);
        boolean closeEnoughForParallel = pivot.angleReached(wantedState.armAngle, 8);
        double nextExtender =
                currentBelowMaxRotateLength ? extender.getNativePos() : kMaxRotationLength - 2048;
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
        wrist.setAngle(wantedState.wristAngle);
        pivot.setAngle(nextPivot);
        extender.setLengthMeters(nextExtender);
    }

    public boolean pivotExtenderReached(SuperstructureState state) {
        return extender.reachedPosition(state.length, 5000)
                && pivot.angleReached(state.armAngle, 1.5)
                && wrist.angleReached(state.wristAngle, 2);
    }

    public Command scoreAndStow(double secsBetweenOpenAndStow) {
        return Commands.sequence(
                new ConditionalCommand(
                                score(() -> getScoringPosition().piece),
                                score(() -> currentGamePiece),
                                () -> this.isAutoSteerEnabled)
                        .withTimeout(0.3),
                Commands.waitSeconds(secsBetweenOpenAndStow),
                stowScore());
    }

    public Command scoreAndStowConeReversed(SuperstructureState nextState) {
        return Commands.sequence(
                rollersCommands.openloop(() -> -0.3).withTimeout(0.2),
                goToStateParallel(nextState));
    }

    public Command scoreAndStowCube() {
        return scoreAndStowCube(0.5, SuperstructureState.stowScore);
    }

    public Command scoreAndStowCube(double timeout, SuperstructureState nextState) {
        return scoreAndStowCube(timeout, -0.6, nextState);
    }

    public Command scoreAndStowCube(double timeout, double demand, SuperstructureState nextState) {
        return Commands.sequence(
                rollersCommands.openloop(() -> demand).withTimeout(timeout),
                goToStateParallel(nextState));
    }

    public Command score(Supplier<GamePiece> piece) {
        return new ConditionalCommand(
                rollersCommands.outCone(),
                rollersCommands.outCube(),
                () -> piece.get() == GamePiece.Cone);
    }

    public Command doubleStationCone() {
        return goToStateParallel(SuperstructureState.doubleStationCone);
    }

    public Command doubleStationCube() {
        return goToStateParallel(SuperstructureState.doubleStationCube);
    }

    public Command groundIntakeCone() {
        return goToStateParallel(SuperstructureState.groundIntakeCone);
    }

    public Command groundIntakeCube() {
        return goToStateParallel(SuperstructureState.groundIntakeCube);
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

    public Command setWantedIntakeGamePieceCommand(GamePiece piece) {
        return Commands.runOnce(() -> this.intakeGamePiece = piece);
    }

    public Command lowerWristOffset() {
        return Commands.runOnce(() -> this.wristAdjust += 2);
    }

    public Command higherWristOffset() {
        return Commands.runOnce(() -> this.wristAdjust -= 2);
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

    public void setWantedIntakeGamePiece(GamePiece piece) {
        this.intakeGamePiece = piece;
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
                        < 1;
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
        this.drivetrainWantMove = drivetrainWantMove;
    }

    private final Compressor compressor;
    private final SwerveDrive drive;
    private final Pivot pivot;
    public final Extender extender;
    private final Rollers rollers;
    private final Wrist wrist;
    private final GroupPrinter printer = GroupPrinter.getInstance();
    private final PositionFinder finder;
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
