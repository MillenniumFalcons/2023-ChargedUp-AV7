package team3647.frc2023.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import team3647.frc2023.constants.FieldConstants;
import team3647.frc2023.constants.LimelightConstant;
import team3647.frc2023.robot.PositionFinder.GamePiece;
import team3647.frc2023.subsystems.SwerveDrive;
import team3647.lib.vision.LimelightHelpers;

public class DrivetrainCommands {

    public Command balance(PIDController pitchController, PIDController rollController) {

        return Commands.runEnd(
                () -> {
                    if (swerve.isBalanced(3)) {
                        return;
                    } else {

                        double pitchPID = pitchController.calculate(swerve.getPitch(), -1.71);
                        double rollPID = rollController.calculate(-swerve.getRoll(), 2.15);

                        // SmartDashboard.putNumber("PITCH PID", pitchPID);
                        // SmartDashboard.putNumber("ROLL PID", rollPID);
                        // swerve.drive(new Translation2d(rollPID, pitchPID), 0, false, false);
                    }
                },
                swerve::stopModules,
                swerve);
    }

    public Command driveVisionTeleop(
            DoubleSupplier xSpeedFunction,
            DoubleSupplier ySpeedFunction,
            DoubleSupplier turnSpeedFunction,
            Supplier<Twist2d> supplier,
            BooleanSupplier shouldLockY,
            BooleanSupplier shouldLockRotation,
            BooleanSupplier triggerSlowMode,
            BooleanSupplier shouldCorrect) {
        return Commands.run(
                () -> {
                    double triggerSlow = triggerSlowMode.getAsBoolean() ? 0.5 : 1;
                    boolean lockY = shouldLockY.getAsBoolean();
                    boolean lockRotation = shouldLockRotation.getAsBoolean();
                    var motionXComponent = ySpeedFunction.getAsDouble() * maxSpeed * triggerSlow;
                    // right stick X, (negative so that left positive)
                    var motionYComponent = -xSpeedFunction.getAsDouble() * maxSpeed * triggerSlow;

                    var motionTurnComponent =
                            -turnSpeedFunction.getAsDouble() * maxRotationRadpS * triggerSlow;
                    motionYComponent = lockY ? supplier.get().dy : motionYComponent;
                    motionTurnComponent =
                            lockRotation ? supplier.get().dtheta : motionTurnComponent;
                    boolean correct = shouldCorrect.getAsBoolean();
                    Translation2d translation =
                            new Translation2d(motionXComponent, motionYComponent);
                    swerve.drive(translation, motionTurnComponent, true, true, correct);
                },
                swerve);
    }

    public Command greenLightAim(
            PIDController strafeController,
            PIDController turnController,
            Supplier<GamePiece> getCurrentGamePiece,
            DoubleSupplier ySpeed,
            DoubleSupplier xSpeed,
            DoubleSupplier rotSpeed) {
        strafeController.setTolerance(2);
        turnController.setTolerance(2);
        Command align =
                new RunCommand(
                        () -> {
                            double tx =
                                    -LimelightHelpers.getTX(LimelightConstant.kLimelightCenterHost);
                            double error =
                                    swerve.getOdoRot()
                                            .rotateBy(FieldConstants.kOneEighty)
                                            .getDegrees();

                            double turnDemand = turnController.calculate(error);

                            turnDemand = 1 * -rotSpeed.getAsDouble() + 0.8 * turnDemand;

                            boolean turnDone = Math.abs(turnDemand) < 0.01;
                            boolean turnAlmostDone = Math.abs(turnDemand) < 0.1;

                            if (turnDone) {
                                turnDemand = 0;
                            }

                            double strafeDemand = strafeController.calculate(tx, 0);
                            double ySpeedDemand =
                                    Math.abs(ySpeed.getAsDouble()) > 0.3 ? ySpeed.getAsDouble() : 0;

                            if (Math.abs(strafeDemand) < 0.1 || !turnAlmostDone) {
                                strafeDemand = 0;
                            }
                            strafeDemand = 1.4 * -ySpeedDemand + 1.0 * strafeDemand;

                            if (getCurrentGamePiece.get() == GamePiece.Cube) {
                                strafeDemand = 0;
                                turnDemand = 0;
                            }

                            swerve.drive(
                                    new Translation2d(
                                            xSpeed.getAsDouble() * maxSpeed, strafeDemand),
                                    turnDemand,
                                    true,
                                    false,
                                    false);
                        },
                        swerve);
        Command ledOn =
                new InstantCommand(
                        () ->
                                LimelightHelpers.setLEDMode_ForceOn(
                                        LimelightConstant.kLimelightCenterHost));

        return new SequentialCommandGroup(ledOn, align);
    }

    public Command robotRelativeDrive(Translation2d t, Rotation2d rotation, double seconds) {
        return Commands.run(() -> swerve.drive(t, rotation.getDegrees(), false, true, true), swerve)
                .finallyDo(interupted -> swerve.end())
                .withTimeout(seconds);
    }

    private final SwerveDrive swerve;
    private static final Rotation2d OneEightyRotation = FieldConstants.kOneEighty;
    private final double maxSpeed;
    private final double maxRotationRadpS;

    public DrivetrainCommands(SwerveDrive swerve) {
        this.swerve = swerve;
        this.maxSpeed = this.swerve.getMaxSpeedMpS();
        this.maxRotationRadpS = this.swerve.getMaxRotationRadpS();
    }
}
