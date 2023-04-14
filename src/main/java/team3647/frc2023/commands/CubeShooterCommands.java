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

    public Command intake() {
        return Commands.parallel(
                Commands.run(() -> cubeShooterTop.openLoop(-0.5), cubeShooterTop),
                Commands.run(() -> cubeShooterBottom.openLoop(0.5), cubeShooterBottom),
                Commands.run(() -> cubeWrist.setAngle(70), cubeWrist));
    }

    public Command scoreHybrid() {
        return Commands.parallel(
                Commands.run(() -> cubeShooterTop.openLoop(0.5), cubeShooterTop),
                Commands.run(() -> cubeShooterBottom.openLoop(-0.5), cubeShooterBottom),
                Commands.run(() -> cubeWrist.setAngle(60), cubeWrist));
    }

    public Command scoreMid() {
        return Commands.parallel(
                Commands.run(() -> cubeShooterTop.openLoop(0.5), cubeShooterTop),
                Commands.run(() -> cubeShooterBottom.openLoop(-0.5), cubeShooterBottom),
                Commands.run(() -> cubeWrist.setAngle(60), cubeWrist));
    }

    public Command scoreHigh() {
        return Commands.parallel(
                Commands.run(() -> cubeShooterTop.openLoop(0.5), cubeShooterTop),
                Commands.run(() -> cubeShooterBottom.openLoop(-0.5), cubeShooterBottom),
                Commands.run(() -> cubeWrist.setAngle(60), cubeWrist));
    }

    public Command yeetAcrossCS() {
        return Commands.parallel(
                Commands.run(() -> cubeShooterTop.openLoop(1.0), cubeShooterTop),
                Commands.run(() -> cubeShooterBottom.openLoop(1.0), cubeShooterBottom),
                Commands.run(() -> cubeWrist.setAngle(0), cubeWrist));
    }

    public Command stow() {
        return Commands.parallel(
                Commands.run(() -> cubeShooterTop.openLoop(0), cubeShooterTop),
                Commands.run(() -> cubeShooterBottom.openLoop(0), cubeShooterBottom),
                Commands.run(() -> cubeWrist.setAngle(0), cubeWrist));
    }

    public Command holdPositionAtCall() {
        return new Command() {
            double degreeAtStart = CubeWristConstants.kInitialDegree;

            @Override
            public void initialize() {
                degreeAtStart = cubeWrist.getAngle();
                System.out.println("Initialized: " + degreeAtStart);
            }

            @Override
            public void execute() {
                cubeWrist.setAngle(degreeAtStart);
            }

            @Override
            public Set<Subsystem> getRequirements() {
                return requirements;
            }
        };
    }

    private final Set<Subsystem> requirements;

    public CubeShooterCommands(
            CubeShooterTop cubeShooterTop,
            CubeShooterBottom cubeShooterBottom,
            CubeWrist cubeWrist) {
        this.cubeShooterBottom = cubeShooterBottom;
        this.cubeShooterTop = cubeShooterTop;
        this.cubeWrist = cubeWrist;
        this.requirements = Set.of(cubeShooterBottom, cubeShooterTop, cubeWrist);
    }
}
