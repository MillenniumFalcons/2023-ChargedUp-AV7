package team3647.frc2023.subsystems;

import com.google.common.base.Supplier;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
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

    public Command driveAndArmParallel(Supplier<PathPoint> getPoint, Supplier<Level> getLevel) {
        return Commands.parallel(driveToScore(getPoint), arm(getLevel));
    }

    public Command driveAndArmSequential(Supplier<PathPoint> getPoint, Supplier<Level> getLevel) {
        return driveToScore(getPoint).andThen(arm(getLevel));
    }

    public Command arm(Supplier<Level> getLevel) {
        return new ProxyCommand(() -> goToLevel(getLevel.get()));
    }

    public Command driveToScore(Supplier<PathPoint> getPoint) {
        return drivetrainCommands.toPointCommand(getPoint);
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

    public Command groundIntake() {
        return Commands.parallel(
                pivotCommands.setAngle(() -> Level.groundIntake.angle),
                extenderCommands.length(() -> Level.groundIntake.length));
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

        private Level(double angle, double length, String name) {
            this.angle = angle;
            this.length = length;
            this.name = name;
        }

        public static final Level coneOne =
                new Level(141.44, ExtenderConstants.kMinimumPositionMeters, "cone low");
        public static final Level coneTwo =
                new Level(141.44, ExtenderConstants.kLevelTwoExtendCone, "cone mid");
        public static final Level coneThree =
                new Level(141.44 - 3, ExtenderConstants.kLevelThreeExtendCone, "cone high");

        public static final Level cubeOneReversed =
                new Level(35, ExtenderConstants.kMinimumPositionMeters, "cube reversed low");
        public static final Level cubeTwoReversed =
                new Level(27, ExtenderConstants.kLevelTwoExtendCube, "cube reversed mid");
        public static final Level cubeThreeReversed =
                new Level(39, ExtenderConstants.kLevelThreeExtendCube, "cube reversed high");

        public static final Level cubeOne =
                new Level(145, ExtenderConstants.kMinimumPositionMeters, "cube low");
        public static final Level cubeTwo =
                new Level(153, ExtenderConstants.kLevelTwoExtendCube, "cube mid");
        public static final Level cubeThree =
                new Level(147 - 6, ExtenderConstants.kLevelThreeExtendCube, "cube high");

        public static final Level noLevel =
                new Level(
                        PivotConstants.kInitialAngle,
                        ExtenderConstants.kMinimumPositionMeters,
                        "no level");

        public static final Level groundIntake =
                new Level(189, ExtenderConstants.kMinimumPositionMeters, "ground intake");

        public static final Level station =
                new Level(139, ExtenderConstants.kMinimumPositionMeters, "station");
    }
}
