// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3647.frc2023.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.Set;
import java.util.function.DoubleSupplier;
import team3647.frc2023.constants.WristConstants;
import team3647.frc2023.subsystems.Wrist;

/** Add your docs here. */
public class WristCommands {

    public Command openloop(DoubleSupplier demand) {
        return Commands.run(() -> wrist.setOpenloop(demand.getAsDouble()), wrist);
    }

    public Command setAngle(double angle) {
        return Commands.run(() -> wrist.setAngle(angle), wrist);
    }

    public Command holdPositionAtCall() {
        return new Command() {
            double degreeAtStart = WristConstants.kInitialDegree;

            @Override
            public void initialize() {
                degreeAtStart = wrist.getAngle();
                System.out.println("Initialized: " + degreeAtStart);
            }

            @Override
            public void execute() {
                wrist.setAngle(degreeAtStart);
            }

            @Override
            public Set<Subsystem> getRequirements() {
                return requirements;
            }
        };
    }

    private final Wrist wrist;
    private final Set<Subsystem> requirements;

    public WristCommands(Wrist wrist) {
        this.wrist = wrist;
        this.requirements = Set.of(wrist);
    }
}
