package team3647.frc2023.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.List;
import java.util.Map;
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
    private SuperstructureState currentState = SuperstructureState.stow;
    private List<ScoringPosition> scoringPositions =
            List.of(ScoringPosition.kNone, ScoringPosition.kNone, ScoringPosition.kNone);
    private ScoringPosition scoringPositionBySide = ScoringPosition.kNone;

    public void periodic(double timestamp) {
        scoringPositions = finder.getScoringPositions();
        scoringPositionBySide = finder.getPositionBySide(getWantedSide());
        if (getWantedLevel() == Level.Ground) {
            // shift pose into the field so we don't kill the arm (+x if blue alliance, -x if red
            // alliance)
        }
        SmartDashboard.putString(
                "Game Piece", scoringPositionBySide.piece == GamePiece.Cone ? "CONE" : "CUBE");
    }

    public Command scoreStowHalfSecDelay() {
        return scoreAndStow(0.5);
    }

    public Command armAutomatic() {
        return goToStateParallel(
                () ->
                        finder.getSuperstructureStateByPiece(
                                getWantedLevel(), getScoringPosition().piece));
    }

    public Command intakeAutomatic() {
        return Commands.parallel(
                Commands.select(
                                Map.of(
                                        StationType.Double,
                                        Commands.parallel(
                                                rollersCommands.intakeCone(), doubleStation()),
                                        StationType.Ground,
                                        Commands.parallel(
                                                rollersCommands.intakeCube(), groundIntake())),
                                this::getWantedStation)
                        .until(recheckIntakeMode)
                        .repeatedly());
    }

    public Command stowFromIntake() {
        return Commands.parallel(
                        rollersCommands.intakeCone().withTimeout(1),
                        Commands.select(
                                Map.of(
                                        StationType.Single,
                                        new WaitCommand(0.5).andThen(stow()),
                                        StationType.Double,
                                        new WaitCommand(0.5).andThen(stow()),
                                        StationType.Ground,
                                        new WaitCommand(0.5).andThen(stow())),
                                this::getWantedStation))
                .andThen(rollersCommands.intakeCone().withTimeout(0.5));
    }

    public Command armToPieceFromSide() {
        return goToStateParallel(
                () ->
                        finder.getSuperstructureStateByPiece(
                                getWantedLevel(),
                                getWantedSide() == Side.Center ? GamePiece.Cube : GamePiece.Cone));
    }

    public Command armCone() {
        return goToStateParallel(
                () -> finder.getSuperstructureStateByPiece(getWantedLevel(), GamePiece.Cone));
    }

    public Command armCube() {
        return goToStateParallel(
                () -> finder.getSuperstructureStateByPiece(getWantedLevel(), GamePiece.Cone));
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
                pivotCommands.goDownDegrees(5),
                rollersCommands.outCone().withTimeout(0.2),
                Commands.waitSeconds(secsBetweenOpenAndStow),
                stow());
    }

    public Command scoreAndStowCube(double secsBetweenOpenAndStow) {
        return Commands.sequence(
                pivotCommands.goDownDegrees(5),
                rollersCommands.outCone().withTimeout(1),
                Commands.waitSeconds(secsBetweenOpenAndStow),
                stow());
    }

    public Command singleStation() {
        return goToStateParallel(SuperstructureState.singleStation);
    }

    public Command doubleStation() {
        return goToStateParallel(SuperstructureState.doubleStation);
    }

    public Command groundIntake() {
        return goToStateParallel(SuperstructureState.groundIntake);
    }

    public Command stow() {
        return goToStateParallel(SuperstructureState.stow);
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

    public Command intakeIfArmMoves() {
        return Commands.run(
                () -> {
                    if (Math.abs(pivot.getVelocity()) > 50) {
                        rollers.setOpenloop(-1);
                    } else if (this.drivetrainWantMove.getAsBoolean()) {
                        rollers.setOpenloop(-0.4);
                    } else {
                        rollers.setOpenloop(0);
                    }
                },
                rollers);
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
                && this.scoringPositionBySide
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
            Trigger intakeButtons,
            Trigger chooseSideButtons,
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
        recheckIntakeMode = intakeButtons;
        recheckSide = chooseSideButtons;
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
    private final Trigger recheckIntakeMode;
    private final Trigger recheckSide;
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
