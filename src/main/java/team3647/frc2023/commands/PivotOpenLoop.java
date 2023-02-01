// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3647.frc2023.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.DoubleSupplier;
import team3647.frc2023.subsystems.Pivot;

public class PivotOpenLoop extends CommandBase {
    private final Pivot pivot;
    private final DoubleSupplier output;
    /** Creates a new Pivot. */
    public PivotOpenLoop(Pivot pivot, DoubleSupplier output) {
        this.pivot = pivot;
        this.output = output;
        addRequirements(pivot);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        pivot.setOpenLoop(output.getAsDouble());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
