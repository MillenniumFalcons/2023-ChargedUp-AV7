package team3647.frc2023.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.Set;
import java.util.function.DoubleSupplier;
import team3647.frc2023.constants.PivotConstants;
import team3647.frc2023.subsystems.Pivot;

public class PivotCommands {

    public Command openloop(DoubleSupplier demand) {
        return Commands.run(() -> pivot.setOpenloop(demand.getAsDouble()), this.pivot);
    }

    public Command openLoopConstant(DoubleSupplier demand) {

        return Commands.run(() -> pivot.setOpenloop(0.3 * demand.getAsDouble()), this.pivot);
    }

    public Command setAngle(DoubleSupplier setpoint) {

        return Commands.run(() -> pivot.setAngle(setpoint.getAsDouble()), pivot)
                .withInterruptBehavior(InterruptionBehavior.kCancelSelf)
                .until(() -> true);
    }

    public Command goDownDegrees(double degrees) {
        return new Command() {
            private double goToAngle = 90;

            @Override
            public void initialize() {
                if (pivot.getAngle() > 90) {

                    goToAngle = pivot.getAngle() + degrees;
                } else {
                    goToAngle = pivot.getAngle() - degrees;
                }
            }

            @Override
            public void execute() {
                pivot.setAngle(goToAngle);
            }

            @Override
            public boolean isFinished() {
                return Math.abs(pivot.getAngle() - this.goToAngle) < 0.5;
            }

            @Override
            public Set<Subsystem> getRequirements() {
                return requirements;
            }
        };
    }

    public Command stow() {
        return setAngle(() -> PivotConstants.kInitialAngle).andThen(Commands.run(() -> {}, pivot));
    }

    public Command stowAuto() {
        return setAngle(() -> PivotConstants.kInitialAngle);
    }

    public Command holdPositionAtCall() {
        return new Command() {
            double degreeAtStart = PivotConstants.kInitialAngle;

            @Override
            public void initialize() {
                degreeAtStart = pivot.getAngle();
            }

            @Override
            public void execute() {
                pivot.setAngle(degreeAtStart);
            }

            @Override
            public Set<Subsystem> getRequirements() {
                return requirements;
            }
        };
    }

    private final Pivot pivot;
    private final Set<Subsystem> requirements;

    public PivotCommands(Pivot pivot) {
        this.pivot = pivot;
        this.requirements = Set.of(pivot);
    }
}
