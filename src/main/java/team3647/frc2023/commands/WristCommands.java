// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3647.frc2023.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.DoubleSupplier;
import team3647.frc2023.subsystems.Wrist;

/** Add your docs here. */
public class WristCommands {

    public Command openloop(DoubleSupplier demand) {
        return Commands.run(() -> wrist.setOpenloop(demand.getAsDouble()), wrist);
    }

    public WristCommands(Wrist wrist) {
        this.wrist = wrist;
    }

    private final Wrist wrist;
}
