package team3647.frc2023.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import team3647.frc2023.commands.DrivetrainCommands;
import team3647.frc2023.commands.ExtenderCommands;
import team3647.frc2023.commands.GrabberCommands;
import team3647.frc2023.commands.PivotCommands;
import team3647.frc2023.constants.ExtenderConstants;
import team3647.frc2023.constants.PivotConstants;

public class Superstructure {

    public void periodic(double timestamp) {
        this.kGPivot = PivotConstants.getkGFromLength(extender.getLengthMeters());
    }

    public Command goToLevel(Level level) {
        return Commands.parallel(
                pivotCommands.setAngle(level.angle), extenderCommands.length(level.length));
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

        public static final Level one = new Level(-25, ExtenderConstants.kMinimumPositionMeters);
        public static final Level two = new Level(135, ExtenderConstants.kLevelTwoExtend);
        public static final Level three = new Level(0, ExtenderConstants.kLevelThreeExtend);
        public static final Level station =
                new Level(135, ExtenderConstants.kMinimumPositionMeters);
    }
}
