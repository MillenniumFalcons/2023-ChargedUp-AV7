package team3647.frc2023.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import team3647.frc2023.subsystems.SwerveDrive;

public class DrivetrainCommands {

    public Command balance(PIDController pitchController, PIDController rollController) {

        return Commands.runEnd(
                () -> {
                    if (swerve.isBalanced(3)) {
                        return;
                    } else {

                        double pitchPID = pitchController.calculate(swerve.getPitch(), -1.71);
                        double rollPID = rollController.calculate(-swerve.getRoll(), 2.15);

                        SmartDashboard.putNumber("PITCH PID", pitchPID);
                        SmartDashboard.putNumber("ROLL PID", rollPID);
                        swerve.drive(new Translation2d(rollPID, pitchPID), 0, false, false);
                    }
                },
                swerve::stopModules,
                swerve);
    }

    public Command drive(
            DoubleSupplier xSpeedFunction, // X axis on joystick is Left/Right
            DoubleSupplier ySpeedFunction, // Y axis on Joystick is Front/Back
            DoubleSupplier turnSpeedFunction,
            DoubleSupplier slowTriggerFunction,
            BooleanSupplier getIsFieldOriented,
            BooleanSupplier shouldFlip) {
        return Commands.run(
                () -> {
                    double triggerSlow =
                            (Math.abs(slowTriggerFunction.getAsDouble()) > 0.5) ? 0.2 : 1;
                    var translation =
                            new Translation2d(
                                            ySpeedFunction.getAsDouble(),
                                            -xSpeedFunction.getAsDouble())
                                    .times(swerve.getMaxSpeedMpS())
                                    .times(triggerSlow);
                    translation =
                            shouldFlip.getAsBoolean()
                                    ? translation.rotateBy(OneEightyRotation)
                                    : translation;
                    var rotation =
                            -turnSpeedFunction.getAsDouble()
                                    * swerve.getMaxRotationRadpS()
                                    * triggerSlow;
                    SmartDashboard.putNumber("Swerve wanted x", translation.getX());
                    SmartDashboard.putNumber("Swerve wanted y", translation.getY());
                    swerve.drive(translation, rotation, getIsFieldOriented.getAsBoolean(), true);
                },
                swerve);
    }

    public Command robotRelativeDrive(Translation2d t, double seconds) {
        return Commands.run(() -> swerve.drive(t, 0, false, true), swerve)
                .finallyDo(interupted -> swerve.end())
                .withTimeout(seconds);
    }

    public Command toPoseCommand(Supplier<Pose2d> getPose) {
        return new ProxyCommand(() -> swerve.toPoseCommand(getPose.get()));
    }

    private static final ProfiledPIDController yController =
            new ProfiledPIDController(5, 0, 0, new Constraints(1, 1));

    private static final ProfiledPIDController xController =
            new ProfiledPIDController(5, 0, 0, new Constraints(1, 1));

    private static final ProfiledPIDController rotationController =
            new ProfiledPIDController(5, 0, 0, new Constraints(10, 3.14));

    public Command toPosePID(Supplier<Pose2d> getPose) {
        return new ProxyCommand(
                () -> {
                    var pose = getPose.get();
                    yController.setGoal(pose.getY());
                    xController.setGoal(pose.getX());
                    xController.setTolerance(0.1);
                    yController.setTolerance(0.1);
                    rotationController.setTolerance(Units.degreesToRadians(10));
                    rotationController.setGoal(pose.getRotation().getRadians());
                    rotationController.enableContinuousInput(-Math.PI, Math.PI);

                    return Commands.run(
                                    () -> {
                                        System.out.println("Wanted Pose: " + pose);
                                        System.out.println(
                                                "current pose: " + swerve.getEstimPose());
                                        var vx =
                                                xController.calculate(swerve.getEstimPose().getX());
                                        var vy =
                                                yController.calculate(swerve.getEstimPose().getY());
                                        var vtheta =
                                                rotationController.calculate(
                                                        swerve.getEstimPose()
                                                                .getRotation()
                                                                .getRadians());
                                        System.out.println("theta speed " + vtheta);
                                        var fieldSpeeds = new ChassisSpeeds(vx, vy, vtheta);
                                        swerve.setFieldRelativeSpeeds(fieldSpeeds);
                                    },
                                    swerve)
                            .until(
                                    () ->
                                            yController.atSetpoint()
                                                    && xController.atSetpoint()
                                                    && rotationController.atSetpoint());
                });
    }

    public Command rotateToTape(PIDController yController, DoubleSupplier angle, double offset) {
        return Commands.run(
                () -> {
                    double distance = Math.sin(Units.degreesToRadians(angle.getAsDouble()));
                    double yPID = yController.calculate(distance, offset);
                    swerve.drive(new Translation2d(0, yPID), 0, false, false);
                },
                swerve);
    }

    private final SwerveDrive swerve;
    private static final Rotation2d OneEightyRotation = Rotation2d.fromDegrees(180);

    public DrivetrainCommands(SwerveDrive swerve) {
        this.swerve = swerve;
    }
}
