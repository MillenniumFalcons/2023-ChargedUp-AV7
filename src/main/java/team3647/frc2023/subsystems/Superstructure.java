package team3647.frc2023.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
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
    }

    public Command scoreStowHalfSecDelay() {
        return scoreAndStow(0.5);
    }

    public Command armAutomatic() {
        return goToStateParallel(() -> finder.getSuperstructureState(getWantedLevel()));
    }

    public Command intakeAutomatic() {
        return Commands.parallel(
                Commands.select(
                                Map.of(
                                        StationType.Single,
                                        Commands.parallel(
                                                rollersCommands.intake(), singleStation()),
                                        StationType.Double,
                                        Commands.parallel(
                                                rollersCommands.intake(), doubleStation()),
                                        StationType.Ground,
                                        Commands.parallel(
                                                rollersCommands.openloop(() -> -0.4),
                                                groundIntake())),
                                this::getWantedStation)
                        .until(recheckIntakeMode)
                        .repeatedly());
    }

    public Command stowFromIntake() {
        return Commands.parallel(
                        rollersCommands.intake().withTimeout(1),
                        Commands.select(
                                Map.of(
                                        StationType.Single,
                                        new WaitCommand(0.5).andThen(stow()),
                                        StationType.Double,
                                        new WaitCommand(0.5).andThen(stow()),
                                        StationType.Ground,
                                        new WaitCommand(0.5).andThen(stow())),
                                this::getWantedStation))
                .andThen(rollersCommands.intake().withTimeout(1));
    }

    public Command armToPieceFromSide() {
        return new ConditionalCommand(armCube(), armCone(), () -> getWantedSide() == Side.Center)
                .until(recheckSide)
                .repeatedly();
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
        return Commands.run(() -> runPivotExtender(state), pivot, extender);
    }

    public Command goToStateParallel(Supplier<SuperstructureState> getState) {
        return Commands.run(() -> runPivotExtender(getState.get()), pivot, extender);
    }

    private final double kMaxRotationLength = 15000;

    private void runPivotExtender(SuperstructureState wantedState) {
        boolean currentBelowMaxRotateLength = extender.getNativePos() < kMaxRotationLength + 500;
        boolean nextBelowMaxRotateLength = wantedState.length < kMaxRotationLength + 500;
        boolean needsRotate = !pivot.angleReached(wantedState.angle, 1.5);
        boolean needsExtend = extender.reachedPosition(wantedState.length, 5000);
        boolean closeEnoughForParallel = pivot.angleReached(wantedState.angle, 8);
        double nextExtender = kMaxRotationLength;
        double nextPivot = pivot.getAngle();

        if (needsRotate && !closeEnoughForParallel) {
            if (currentBelowMaxRotateLength && nextBelowMaxRotateLength) {
                nextExtender = wantedState.length;
                nextPivot = wantedState.angle;
            } else if (currentBelowMaxRotateLength) {
                nextPivot = wantedState.angle;
            }
        } else if (closeEnoughForParallel && !needsExtend) {
            nextPivot = wantedState.angle;
        } else {
            nextExtender = wantedState.length;
        }

        extender.setLengthMeters(nextExtender);
        pivot.setAngle(nextPivot);
    }

    public boolean extenderLengthReached(double extenderLength, double wantedLength) {
        return Math.abs(extenderLength - wantedLength) < 3000;
    }

    public boolean armAngleReached(double armAngle, double aimedAngle) {
        if (armAngle < aimedAngle) {
            return armAngle > aimedAngle * 0.8;
        }
        return aimedAngle < aimedAngle * 1.2;
    }

    public Command scoreAndStow(double secsBetweenOpenAndStow) {
        return Commands.sequence(
                pivotCommands.goDownDegrees(5),
                rollersCommands.out().withTimeout(0.2),
                Commands.waitSeconds(secsBetweenOpenAndStow),
                stow());
    }

    public Command scoreAndStowCube(double secsBetweenOpenAndStow) {
        return Commands.sequence(
                pivotCommands.goDownDegrees(5),
                rollersCommands.out().withTimeout(1),
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
        return goToStateParallel(() -> SuperstructureState.stow);
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

    public boolean validScoringPosition() {
        return this.scoringPositionBySide != ScoringPosition.kNone;
    }

    // keep this at the bottom
    public Superstructure(
            SwerveDrive drive,
            Pivot pivot,
            Extender extender,
            Rollers rollers,
            PositionFinder finder,
            Compressor compressor,
            Trigger intakeButtons,
            Trigger chooseSideButtons,
            BooleanSupplier drivetrainWantMove) {
        this.drive = drive;
        this.pivot = pivot;
        this.extender = extender;
        this.rollers = rollers;
        this.finder = finder;
        this.compressor = compressor;

        drivetrainCommands = new DrivetrainCommands(drive);
        pivotCommands = new PivotCommands(pivot);
        extenderCommands = new ExtenderCommands(extender);
        rollersCommands = new RollersCommands(rollers);
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
    private final GroupPrinter printer = GroupPrinter.getInstance();
    private final PositionFinder finder;
    private final Trigger recheckIntakeMode;
    private final Trigger recheckSide;
    public final DrivetrainCommands drivetrainCommands;
    public final PivotCommands pivotCommands;
    public final ExtenderCommands extenderCommands;
    public final RollersCommands rollersCommands;
    private final BooleanSupplier drivetrainWantMove;

    public enum StationType {
        Single,
        Double,
        Ground
    }
}
