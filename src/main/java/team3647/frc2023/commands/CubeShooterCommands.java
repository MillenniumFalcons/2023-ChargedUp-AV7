package team3647.frc2023.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.Set;
import team3647.frc2023.constants.CubeWristConstants;
import team3647.frc2023.subsystems.CubeShooterBottom;
import team3647.frc2023.subsystems.CubeShooterTop;
import team3647.frc2023.subsystems.CubeWrist;

public class CubeShooterCommands {
    private final CubeShooterTop cubeShooterTop;
    private final CubeShooterBottom cubeShooterBottom;
    private final CubeWrist cubeWrist;

    public Command cubeWristOpenloop(double demand) {
        return Commands.run(() -> cubeWrist.setOpenloop(demand), cubeWrist);
    }

    public Command cubeShooterRun(double angle, double topDemand, double bottomDemand) {
        return Commands.sequence(
                Commands.run(() -> cubeWrist.setAngle(angle), cubeWrist)
                        .until(() -> cubeWrist.angleReached(angle, 5)),
                Commands.parallel(
                        Commands.runEnd(
                                () -> cubeShooterBottom.openLoop(bottomDemand),
                                cubeShooterBottom::end,
                                cubeShooterBottom),
                        Commands.runEnd(
                                () -> cubeShooterTop.openLoop(topDemand),
                                cubeShooterTop::end,
                                cubeShooterTop)));
    }

    public Command intake() {
        return cubeShooterRun(91, -0.8, -0.8);
    }

    public Command scoreHybrid() {
        return cubeShooterRun(80, 0.6, 0.6);
    }

    public Command scoreMid() {
        return cubeShooterRun(5, 1, 1);
    }

    public Command scoreHigh() {
        return cubeShooterRun(2, 0.8, 1);
    }

    public Command yeetAcrossCS() {
        return Commands.parallel(
                Commands.run(() -> cubeShooterTop.openLoop(1.0), cubeShooterTop),
                Commands.run(() -> cubeShooterBottom.openLoop(1.0), cubeShooterBottom),
                Commands.run(() -> cubeWrist.setAngle(0), cubeWrist));
    }

    public Command stow() {
        return cubeShooterRun(2, 0, 0).until(() -> cubeWrist.angleReached(2, 5));
    }

    public Command holdPositionAtCall() {
        return new Command() {
            double degreeAtStart = CubeWristConstants.kInitialDegree;

            @Override
            public void initialize() {
                degreeAtStart = cubeWrist.getAngle();
            }

            @Override
            public void execute() {
                cubeWrist.setAngle(degreeAtStart);
            }

            @Override
            public Set<Subsystem> getRequirements() {
                return requirementsWrist;
            }
        };
    }

    private final Set<Subsystem> requirements;
    private final Set<Subsystem> requirementsWrist;

    public CubeShooterCommands(
            CubeShooterTop cubeShooterTop,
            CubeShooterBottom cubeShooterBottom,
            CubeWrist cubeWrist) {
        this.cubeShooterBottom = cubeShooterBottom;
        this.cubeShooterTop = cubeShooterTop;
        this.cubeWrist = cubeWrist;
        this.requirements = Set.of(cubeShooterBottom, cubeShooterTop, cubeWrist);
        this.requirementsWrist = Set.of(cubeWrist);
    }
}
