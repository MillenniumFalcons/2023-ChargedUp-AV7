package team3647.frc2023.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import java.util.Objects;
import java.util.function.Supplier;
import team3647.frc2023.commands.DrivetrainCommands;
import team3647.frc2023.commands.ExtenderCommands;
import team3647.frc2023.commands.GrabberCommands;
import team3647.frc2023.commands.PivotCommands;
import team3647.frc2023.constants.ExtenderConstants;
import team3647.frc2023.constants.PivotConstants;
import team3647.lib.GroupPrinter;

public class Superstructure {

    public void periodic(double timestamp) {
        // this.kGPivot = PivotConstants.getkGFromLength(extender.getLengthMeters());
    }

    public Command driveAndArmParallel(Supplier<Pose2d> getPose, Supplier<Level> getLevel) {
        return Commands.parallel(drivetrainCommands.toPoseCommand(getPose), arm(getLevel));
    }

    public Command driveAndArmSequential(Supplier<Pose2d> getPose, Supplier<Level> getLevel) {
        return new ProxyCommand(
                () -> {
                    var pose = getPose.get();
                    if (Objects.isNull(pose)) {
                        return arm(getLevel);
                    }
                    return drivetrainCommands.toPoseCommand(getPose).andThen(arm(getLevel));
                });
    }

    public Command driveAndArmSequentialPID(Supplier<Pose2d> getPose, Supplier<Level> getLevel) {
        return new ProxyCommand(
                () -> {
                    var pose = getPose.get();
                    if (Objects.isNull(pose)) {
                        return arm(getLevel);
                    }
                    return drivetrainCommands.toPosePID(getPose).andThen(arm(getLevel));
                });
    }

    public Command arm(Supplier<Level> getLevel) {
        return new ProxyCommand(() -> goToLevel(getLevel.get()));
    }

    public Command armAuto(Supplier<Level> getLevel) {
        Level level = getLevel.get();
        return Commands.parallel(
                pivotCommands.setAngle(() -> level.angle),
                Commands.waitUntil(() -> pivot.getAngle() > level.angle * 0.5)
                        .andThen(extenderCommands.length(() -> level.length)));
        // .until(
        //         () ->
        //                 Math.abs(pivot.getAngle() - level.angle) < 2.5
        //                         && Math.abs(extender.getNativeTicks() - level.length)
        //                                 < 2000);
    }

    public Command armRetractAuto(Supplier<Level> getLevel) {
        Level level = getLevel.get();
        return Commands.parallel(
                extenderCommands.length(() -> level.length),
                Commands.waitUntil(
                                () ->
                                        extender.getNativePos()
                                                < ExtenderConstants.kMaximumPositionTicks * 0.2)
                        .andThen(pivotCommands.setAngle(() -> level.angle)));
        // .until(
        //         () ->
        //                 Math.abs(pivot.getAngle() - level.angle) < 2.5
        //                         && Math.abs(extender.getNativeTicks() - level.length)
        //                                 < 2000);
    }

    public Command cancelPivot() {
        return Commands.run(() -> {}, pivot).withTimeout(0.2);
    }

    public Command cancelExtender() {
        return Commands.run(() -> {}, extender).withTimeout(0.2);
    }

    public Command goToLevel(Level level) {

        return Commands.parallel(
                pivotCommands.setAngle(() -> level.angle),
                Commands.waitUntil(() -> pivot.getAngle() > level.angle * 0.5)
                        .andThen(extenderCommands.length(() -> level.length)));
    }

    public Command loadingStation() {
        return Commands.parallel(
                pivotCommands.setAngle(() -> Level.station.angle),
                Commands.waitUntil(() -> pivot.getAngle() > Level.station.angle * 0.9)
                        .andThen(extenderCommands.length(() -> Level.station.length)));
    }

    public Command doubleStation() {
        return Commands.parallel(
                pivotCommands.setAngle(() -> Level.doubleStation.angle),
                Commands.waitUntil(() -> pivot.getAngle() > Level.doubleStation.angle)
                        .andThen(extenderCommands.length(() -> Level.doubleStation.length)));
    }

    public Command armToGroundIntake() {
        return Commands.parallel(
                pivotCommands.setAngle(() -> Level.groundIntake.angle),
                extenderCommands.length(() -> Level.groundIntake.length));
    }

    public Command stow() {
        return Commands.parallel(pivotCommands.stow(), extenderCommands.stow());
    }

    public Command stowAuto() {
        return Commands.parallel(pivotCommands.stowAuto(), extenderCommands.stowAuto());
    }

    public Command disableCompressor() {
        return new InstantCommand(compressor::disable);
    }

    public Command enableCompressor() {
        return new InstantCommand(compressor::enableDigital);
    }

    // keep this at the bottom
    public Superstructure(
            SwerveDrive drive,
            Pivot pivot,
            Extender extender,
            Grabber grabber,
            Compressor compressor) {
        this.drive = drive;
        this.pivot = pivot;
        this.extender = extender;
        this.grabber = grabber;
        this.compressor = compressor;

        drivetrainCommands = new DrivetrainCommands(drive);
        pivotCommands = new PivotCommands(pivot);
        extenderCommands = new ExtenderCommands(extender);
        grabberCommands = new GrabberCommands(grabber);
    }

    private double kGPivot;
    private final Compressor compressor;
    private final SwerveDrive drive;
    private final Pivot pivot;
    private final Extender extender;
    private final Grabber grabber;
    private final GroupPrinter printer = GroupPrinter.getInstance();
    public final DrivetrainCommands drivetrainCommands;
    public final PivotCommands pivotCommands;
    public final ExtenderCommands extenderCommands;
    public final GrabberCommands grabberCommands;

    public static class Level {
        /** degrees */
        public final double angle;

        public final double length;

        public final String name;

        // name parameter is just for debugging purposes

        private Level(double angle, double length, String name) {
            this.angle = angle;
            this.length = length;
            this.name = name;
        }

        public static final Level coneOne =
                new Level(150, ExtenderConstants.kMinimumPositionTicks, "cone low");
        public static final Level coneTwo =
                new Level(139, ExtenderConstants.kLevelTwoExtendCone, "cone mid");
        public static final Level coneThree =
                new Level(141 - 3, ExtenderConstants.kLevelThreeExtendCone, "cone high");

        public static final Level coneOneReversed =
                new Level(180 - 150, ExtenderConstants.kMinimumPositionTicks, "cone reversed low");
        public static final Level coneTwoReversed =
                new Level(180 - 139, ExtenderConstants.kLevelTwoExtendCone, "cone reversed mid");
        public static final Level coneThreeReversed =
                new Level(
                        180 - (141 - 3),
                        ExtenderConstants.kLevelThreeExtendCone,
                        "cone reversed high");

        public static final Level cubeOne =
                new Level(125, ExtenderConstants.kMinimumPositionTicks, "cube low");
        public static final Level cubeTwo =
                new Level(149, ExtenderConstants.kLevelTwoExtendCube, "cube mid");
        public static final Level cubeThree =
                new Level(148, ExtenderConstants.kLevelThreeExtendCube, "cube high");

        public static final Level cubeOneReversed =
                new Level(35, ExtenderConstants.kMinimumPositionTicks, "cube reversed low");
        public static final Level cubeTwoReversed =
                new Level(27, ExtenderConstants.kLevelTwoExtendCube, "cube reversed mid");
        public static final Level cubeThreeReversed =
                new Level(180 - 148, ExtenderConstants.kLevelThreeExtendCube, "cube reversed high");

        public static final Level noLevel =
                new Level(
                        PivotConstants.kInitialAngle,
                        ExtenderConstants.kMinimumPositionTicks,
                        "no level");

        public static final Level groundIntake =
                new Level(190, ExtenderConstants.kMinimumPositionTicks, "ground intake");

        public static final Level station =
                new Level(146, ExtenderConstants.kMinimumPositionTicks, "station");

        public static final Level doubleStation =
                new Level(139.5, ExtenderConstants.kGroundStation, "double station");
    }
}
