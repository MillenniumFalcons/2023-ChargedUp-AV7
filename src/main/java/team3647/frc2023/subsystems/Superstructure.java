package team3647.frc2023.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.Map;
import java.util.function.Supplier;
import team3647.frc2023.commands.DrivetrainCommands;
import team3647.frc2023.commands.ExtenderCommands;
import team3647.frc2023.commands.GrabberCommands;
import team3647.frc2023.commands.PivotCommands;
import team3647.frc2023.robot.PositionFinder;
import team3647.frc2023.robot.PositionFinder.GamePiece;
import team3647.frc2023.robot.PositionFinder.Level;
import team3647.frc2023.util.SuperstructureState;
import team3647.lib.GroupPrinter;

public class Superstructure {

    private Level wantedLevel = Level.Stay;
    private StationType wantedStation = StationType.Double;
    private boolean isAutoSteerEnabled = true;

    public void periodic(double timestamp) {}

    public Command scoreStowHalfSecDelay() {
        return scoreAndStow(0.5);
    }

    public Command armAutomatic() {
        return arm(() -> finder.getSuperstructureState(getWantedLevel())).repeatedly();
    }

    public Command intakeAutomatic() {
        return Commands.parallel(
                grabberCommands.openGrabber(),
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
                grabberCommands.closeGrabber(),
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
        return Commands.parallel(
                pivotCommands.setAngle(() -> getState.get().angle),
                extenderCommands.length(() -> getState.get().length));
    }

    public Command scoreAndStow(double secsBetweenOpenAndStow) {
        return Commands.sequence(
                grabberCommands.openGrabber(), new WaitCommand(secsBetweenOpenAndStow), stow());
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
        return Commands.parallel(pivotCommands.stow(), extenderCommands.stow());
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
            Grabber grabber,
            PositionFinder finder,
            Compressor compressor,
            Trigger intakeButtons) {
        this.drive = drive;
        this.pivot = pivot;
        this.extender = extender;
        this.grabber = grabber;
        this.finder = finder;
        this.compressor = compressor;

        drivetrainCommands = new DrivetrainCommands(drive);
        pivotCommands = new PivotCommands(pivot);
        extenderCommands = new ExtenderCommands(extender);
        grabberCommands = new GrabberCommands(grabber);
        recheckIntakeMode = intakeButtons;
    }

    private double kGPivot;
    private final Compressor compressor;
    private final SwerveDrive drive;
    private final Pivot pivot;
    private final Extender extender;
    private final Grabber grabber;
    private final GroupPrinter printer = GroupPrinter.getInstance();
    private final PositionFinder finder;
    private final Trigger recheckIntakeMode;
    public final DrivetrainCommands drivetrainCommands;
    public final PivotCommands pivotCommands;
    public final ExtenderCommands extenderCommands;
    public final GrabberCommands grabberCommands;

    public enum StationType {
        Single,
        Double,
        Ground
    }
}
