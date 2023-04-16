package team3647.frc2023.util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import team3647.frc2023.constants.LimelightConstant;
import team3647.lib.vision.LimelightHelpers;

public class AutoSteer {

    private Pose2d targetPose = new Pose2d();
    private double lastUpdated = Timer.getFPGATimestamp();

    private final PIDController xController, yController;
    private final PIDController thetaController;
    private final DoubleSupplier txSupplier;
    private final Supplier<Pose2d> drivePose;
    private boolean useHeadingOnly = false;

    private static final Twist2d kNoTwist = new Twist2d();
    private boolean stopped = false;

    public AutoSteer(
            Supplier<Pose2d> drivePose,
            DoubleSupplier txSupplier,
            PIDController xController,
            PIDController yController,
            PIDController thetaController) {
        this.drivePose = drivePose;
        this.txSupplier = txSupplier;
        this.xController = xController;
        this.yController = yController;
        xController.setTolerance(2);
        yController.setTolerance(1);
        this.thetaController = thetaController;
        thetaController.setTolerance(2);
        this.thetaController.enableContinuousInput(-180.0, 180);
    }

    public Twist2d findVelocities() {
        if (stopped) {
            return kNoTwist;
        }

        var currentPose = drivePose.get();
        var dx = 0.0;
        var dy = yController.calculate(txSupplier.getAsDouble());
        var dtheta = 0.8 * thetaController.calculate(currentPose.getRotation().getDegrees());

        if (xController.atSetpoint()) {
            dx = 0.0;
        }
        SmartDashboard.putBoolean("Y AT Setpoint", yController.atSetpoint());
        if (yController.atSetpoint()) {
            dy = 0.0;
        }
        if (thetaController.atSetpoint()) {
            dtheta = 0.0;
        }

        // don't strafe until turned to target
        if (Math.abs(dtheta) > 0.1) {
            dy = 0.0;
        }
        if (Math.abs(dtheta) < 0.01) {
            dtheta = 0.0;
        }
        SmartDashboard.putNumber("TX", txSupplier.getAsDouble());
        SmartDashboard.putNumber("dtheta", dtheta);
        SmartDashboard.putNumber("dy", dy);

        if (useHeadingOnly) {
            return new Twist2d(0.0, 0.0, dtheta);
        }

        return new Twist2d(dx, dy, dtheta);
    }

    public void lockHeading(double headingRads) {
        thetaController.setSetpoint(headingRads);
    }

    public void stop() {
        stopped = false;
    }

    public void initializeSteering() {
        stopped = false;
        useHeadingOnly = false;
        lastUpdated = Timer.getFPGATimestamp();
        yController.setSetpoint(0);
        lockHeading(180);
    }

    public void justHeading(double heading) {
        lockHeading(heading);
        lastUpdated = Timer.getFPGATimestamp();
        useHeadingOnly = true;
    }

    public boolean arrived() {
        return xController.atSetpoint() && yController.atSetpoint() && thetaController.atSetpoint();
    }

    public boolean almostArrived() {
        return LimelightHelpers.getTV(LimelightConstant.kLimelightCenterHost) && !stopped;
    }
}
