package team3647.frc2023.util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.Supplier;

public class AutoSteer {

    private Pose2d targetPose = new Pose2d();
    private double lastUpdated = Timer.getFPGATimestamp();

    private final ProfiledPIDController xController, yController;
    private final PIDController thetaController;
    private final Supplier<Pose2d> drivePose;

    private static final Twist2d kNoTwist = new Twist2d();

    public AutoSteer(
            Supplier<Pose2d> drivePose,
            ProfiledPIDController xController,
            ProfiledPIDController yController,
            PIDController thetaController) {
        this.drivePose = drivePose;
        this.xController = xController;
        this.yController = yController;
        this.thetaController = thetaController;
        this.thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public Twist2d findVelocities() {
        if (Timer.getFPGATimestamp() - lastUpdated > 20) {
            return kNoTwist;
        }
        var currentPose = drivePose.get();
        var dx = xController.calculate(currentPose.getX());
        var dy = yController.calculate(currentPose.getY());
        var dtheta = thetaController.calculate(currentPose.getRotation().getRadians());
        return new Twist2d(dx, dy, dtheta);
    }

    public void lockHeading(double headingRads) {
        thetaController.setSetpoint(headingRads);
    }

    public void initializeSteering(Pose2d pose) {
        this.targetPose = pose;
        lastUpdated = Timer.getFPGATimestamp();
        xController.setGoal(pose.getX());
        yController.setGoal(pose.getY());
        lockHeading(pose.getRotation().getRadians());
    }
}
