package team3647.frc2023.subsystems;

import com.pathplanner.lib.PathPoint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
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
        this.kGPivot = PivotConstants.getkGFromLength(extender.getLengthMeters());
    }

    public Command driveAndArm(Supplier<PathPoint> getPoint, Supplier<Level> getLevel) {
        return Commands.parallel(driveToScore(getPoint), arm(getLevel));
    }

    public Command arm(Supplier<Level> getLevel) {
        return new ProxyCommand(() -> goToLevel(getLevel.get()));
    }

    public Command driveToScore(Supplier<PathPoint> getPoint) {
        return drivetrainCommands.toPointCommand(getPoint);
    }

    public Command goToLevel(Level level) {
        return Commands.parallel(
                pivotCommands.setAngle(level.angle),
                Commands.waitUntil(() -> pivot.getAngle() > level.angle * 0.5)
                        .andThen(extenderCommands.length(level.length)));
    }

    public Command loadingStation() {
        return Commands.parallel(
                        grabberCommands.setAngle(100),
                        pivotCommands.setAngle(Level.station.angle),
                        Commands.waitUntil(() -> pivot.getAngle() > Level.station.angle * 0.9)
                                .andThen(extenderCommands.length(Level.station.length)))
                .finallyDo(
                        interrupted -> Commands.run(() -> {}, pivot).withTimeout(0.5).schedule());
    }

    // keep this at the bottom
    public Superstructure(SwerveDrive drive, Pivot pivot, Extender extender, Grabber grabber) {
        this.drive = drive;
        this.pivot = pivot;
        this.extender = extender;
        this.grabber = grabber;

        drivetrainCommands = new DrivetrainCommands(drive);
        pivotCommands = new PivotCommands(pivot);
        extenderCommands = new ExtenderCommands(extender);
        grabberCommands = new GrabberCommands(grabber);
    }

    private double kGPivot;
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

        private Level(double angle, double length) {
            this.angle = angle;
            this.length = length;
        }

        public static final Level coneOne = new Level(-6, ExtenderConstants.kMinimumPositionMeters);
        public static final Level coneTwo = new Level(36, ExtenderConstants.kLevelTwoExtendCone);
        public static final Level coneThree =
                new Level(40, ExtenderConstants.kLevelThreeExtendCone);

        public static final Level cubeOne = new Level(-6, ExtenderConstants.kMinimumPositionMeters);
        public static final Level cubeTwo = new Level(154, ExtenderConstants.kLevelTwoExtendCube);
        public static final Level cubeThree =
                new Level(147, ExtenderConstants.kLevelThreeExtendCube);

        public static final Level station =
                new Level(139, ExtenderConstants.kMinimumPositionMeters);
    }
}
