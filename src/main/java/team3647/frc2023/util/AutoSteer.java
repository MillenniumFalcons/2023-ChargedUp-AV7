package team3647.frc2023.util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.function.Supplier;

public class AutoSteer {

    private Pose2d targetPose = new Pose2d();
    private double lastUpdated = Timer.getFPGATimestamp();

    private final PIDController xController, yController;
    private final PIDController thetaController;
    private final Supplier<Pose2d> drivePose;

    private static final Twist2d kNoTwist = new Twist2d();

    public AutoSteer(
            Supplier<Pose2d> drivePose,
            PIDController xController,
            PIDController yController,
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
        xController.setSetpoint(pose.getX());
        yController.setSetpoint(pose.getY());
        lockHeading(pose.getRotation().getRadians());
    }

    public boolean arrived() {
        return xController.atSetpoint() && yController.atSetpoint() && thetaController.atSetpoint();
    }

    public boolean almostArrived() {
        SmartDashboard.putNumber(
                "Distance to target",
                this.targetPose.minus(drivePose.get()).getTranslation().getNorm());
        return this.targetPose.minus(drivePose.get()).getTranslation().getNorm() < 0.50;
    }
}
