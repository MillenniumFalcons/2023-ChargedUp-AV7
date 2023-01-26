// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3647.frc2023.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team3647.frc2023.constants.SwerveDriveConstants;
import team3647.frc2023.subsystems.SwerveDrive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class Balance extends CommandBase {
    private final SwerveDrive swerve;
    private final SlewRateLimiter m_x_accelerationLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_y_accelerationLimiter = new SlewRateLimiter(3);

    private final BooleanSupplier getIsFieldOriented;

    private final PIDController rollControl = SwerveDriveConstants.krollController;

    private double rotation;
    private Translation2d translation;

    private final double translateMultiplier = 1.0; // 0.1;
    private final double rotationMultiplier = 1.0; // 0.1;

    /** Creates a new SwerveDriveTeleopBaseFalcon. */
    public Balance(
            SwerveDrive swerve,
            BooleanSupplier getIsFieldOriented) {
        this.swerve = swerve;
        this.getIsFieldOriented = getIsFieldOriented;
        addRequirements(swerve);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        translation = new Translation2d(-rollControl.calculate(swerve.getRoll(), 0), 0);
        swerve.drive(translation, rotation, getIsFieldOriented.getAsBoolean(), false);
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
