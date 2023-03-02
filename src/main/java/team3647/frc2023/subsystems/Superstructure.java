package team3647.frc2023.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
import team3647.frc2023.util.SuperstructureState;
import team3647.lib.GroupPrinter;

public class Superstructure {

    private Level wantedLevel = Level.Stay;
    private StationType wantedStation = StationType.Double;
    private boolean isAutoSteerEnabled = true;
    private SuperstructureState currentState = SuperstructureState.stow;

    public void periodic(double timestamp) {}

    public Command scoreStowHalfSecDelay() {
        return scoreAndStow(0.5);
    }

    public Command armAutomatic() {
        return arm(() -> finder.getSuperstructureState(getWantedLevel())).repeatedly();
    }

    public Command intakeAutomatic() {
        return Commands.parallel(
                rollersCommands.intake(),
                Commands.select(
                                Map.of(
                                        StationType.Single,
                                        singleStation(),
                                        StationType.Double,
                                        doubleStation(),
                                        StationType.Ground,
                                        groundIntake()),
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
                        this::getWantedStation));
    }

    public Command arm(Supplier<SuperstructureState> getState) {
        return goToStateParallel(getState);
    }

    public Command armCone() {
        return goToStateParallel(
                () -> finder.getSuperstructureStateByPiece(getWantedLevel(), GamePiece.Cone));
    }

    public Command cancelPivot() {
        return Commands.runOnce(() -> {}, pivot);
    }

    public Command cancelExtender() {
        return Commands.runOnce(() -> {}, extender);
    }

    public Command goToStateArmFirst(SuperstructureState state, double percentDoneRotating) {
        return Commands.parallel(
                pivotCommands.setAngle(() -> state.angle),
                Commands.waitUntil(() -> pivot.getAngle() > state.angle * percentDoneRotating)
                        .andThen(extenderCommands.length(() -> state.length)));
    }

    public Command goToStateParallel(SuperstructureState state) {
        return Commands.parallel(
                pivotCommands.setAngle(() -> state.angle),
                extenderCommands.length(() -> state.length));
    }

    public Command goToStateParallel(Supplier<SuperstructureState> getState) {
        BooleanSupplier stateChanged =
                () -> {
                    boolean stateDifferent = this.currentState != getState.get();
                    this.currentState = getState.get();
                    return stateDifferent;
                };
        return new ConditionalCommand(
                        Commands.parallel(
                                pivotCommands.setAngle(() -> getState.get().angle),
                                Commands.waitUntil(
                                                () ->
                                                        armAngleReached(
                                                                pivot.getAngle(),
                                                                getState.get().angle))
                                        .andThen(
                                                extenderCommands.length(
                                                        () -> getState.get().length))),
                        Commands.parallel(
                                Commands.waitUntil(
                                                () ->
                                                        extenderLengthReached(
                                                                extender.getNativePos(),
                                                                getState.get().length))
                                        .andThen(
                                                pivotCommands.setAngle(() -> getState.get().angle)),
                                extenderCommands.length(() -> getState.get().length)),
                        () -> {
                            var angle = getState.get().angle;
                            var length = getState.get().length;
                            return length > extender.getNativePos();
                        })
                .until(stateChanged)
                .repeatedly();
    }

    public boolean extenderLengthReached(double extenderLength, double wantedLength) {
        return Math.abs(extenderLength - wantedLength) < 1000;
    }

    public boolean armAngleReached(double armAngle, double aimedAngle) {
        if (armAngle < aimedAngle) {
            return armAngle > aimedAngle * 0.8;
        }
        return aimedAngle < aimedAngle * 1.2;
    }

    public Command scoreAndStow(double secsBetweenOpenAndStow) {
        return Commands.sequence(
                pivotCommands.goDownDegrees(5), rollersCommands.out().withTimeout(0.3), stow());
    }

    public Command singleStation() {
        return goToStateArmFirst(SuperstructureState.singleStation, 0.9);
    }

    public Command doubleStation() {
        return goToStateArmFirst(SuperstructureState.doubleStation, 0.9);
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

    public Command enableAutoSteer() {
        return Commands.runOnce(() -> this.isAutoSteerEnabled = true);
    }

    public Command disableAutoSteer() {
        return Commands.runOnce(() -> this.isAutoSteerEnabled = false);
    }

    public Command intakeIfArmMoves() {
        return Commands.run(
                () -> {
                    if (Math.abs(pivot.getVelocity()) < 50) {
                        rollers.setOpenloop(0);
                    } else {
                        rollers.setOpenloop(-0.1);
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

    public boolean autoSteerEnabled() {
        return this.isAutoSteerEnabled;
    }

    // keep this at the bottom
    public Superstructure(
            SwerveDrive drive,
            Pivot pivot,
            Extender extender,
            Rollers rollers,
            PositionFinder finder,
            Compressor compressor,
            Trigger intakeButtons) {
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
    public final DrivetrainCommands drivetrainCommands;
    public final PivotCommands pivotCommands;
    public final ExtenderCommands extenderCommands;
    public final RollersCommands rollersCommands;

    public enum StationType {
        Single,
        Double,
        Ground
    }
}
