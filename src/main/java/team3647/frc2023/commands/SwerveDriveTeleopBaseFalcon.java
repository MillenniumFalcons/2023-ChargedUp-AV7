// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3647.frc2023.commands;


import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team3647.frc2023.constants.SwerveDriveConstants;
import team3647.frc2023.subsystems.SwerveDrive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class SwerveDriveTeleopBaseFalcon extends CommandBase {
    private final SwerveDrive swerve;

    private final DoubleSupplier xSpeedFunction;
    private final DoubleSupplier ySpeedFunction;
    private final DoubleSupplier turnSpeedFunction;
    private final DoubleSupplier aimFunction;
    private final BooleanSupplier getIsFieldOriented;

    private Translation2d arbitoryTarget = new Translation2d(4.0, 4.0);

    private double rotation;
    private Translation2d translation;

    private final double translateMultiplier = 1.0; // 0.1;
    private final double rotationMultiplier = 1.0; // 0.1;

    /** Creates a new SwerveDriveTeleopBaseFalcon. */
    public SwerveDriveTeleopBaseFalcon(
            SwerveDrive swerve,
            DoubleSupplier xSpeedFunction,
            DoubleSupplier ySpeedFunction,
            DoubleSupplier turnSpeedFunction,
            DoubleSupplier aimFunction,
            BooleanSupplier getIsFieldOriented) {
        this.swerve = swerve;
        this.xSpeedFunction = xSpeedFunction;
        this.ySpeedFunction = ySpeedFunction;
        this.turnSpeedFunction = turnSpeedFunction;
        this.aimFunction = aimFunction;
        this.getIsFieldOriented = getIsFieldOriented;
        addRequirements(swerve);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double xComponent = ySpeedFunction.getAsDouble() * translateMultiplier;
        double yComponent = -xSpeedFunction.getAsDouble() * translateMultiplier;
        translation =
                new Translation2d(xComponent, yComponent)
                        .times(SwerveDriveConstants.kDrivePossibleMaxSpeedMPS);
        rotation =
                -turnSpeedFunction.getAsDouble()
                        * SwerveDriveConstants.kRotPossibleMaxSpeedRadPerSec
                        * rotationMultiplier;
        if (aimFunction.getAsDouble() > 0.6) {
            rotation =
                    -Units.degreesToRadians(
                                    optimalAngle(
                                            this.swerve.getHeading(),
                                            angleBetweenPointsInDeg(
                                                    this.swerve.getPose().getTranslation(),
                                                    this.arbitoryTarget)))
                            * 5;
        }
        swerve.drive(translation, rotation, getIsFieldOriented.getAsBoolean(), true);
    }

    private double angleBetweenPointsInDeg(Translation2d point1, Translation2d point2) {
        double dx = point1.getX() - point2.getX();
        double dy = point1.getY() - point2.getY();
        double angle = Units.radiansToDegrees(Math.atan(dy / dx));
        if (this.swerve.getPose().getX() > this.arbitoryTarget.getX()) {
            if (this.swerve.getPose().getY() > this.arbitoryTarget.getY()) {
                angle -= 180;
            } else {
                angle += 180;
            }
        }
        return angle;
    }

    private double optimalAngle(double angle, double angleToGo) {
        double turnToAngle =
                swerve.getHeading()
                        - angleBetweenPointsInDeg(
                                swerve.getPose().getTranslation(), this.arbitoryTarget);
        if (turnToAngle > 180) {
            turnToAngle -= 360;
        }
        if (turnToAngle < -180) {
            turnToAngle += 360;
        }
        return turnToAngle;
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
