package team3647.frc2023.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import team3647.lib.PeriodicSubsystem;
import team3647.lib.SwerveModule;

public class SwerveDrive implements PeriodicSubsystem {
    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backLeft;
    private final SwerveModule backRight;

    private final SwerveDriveKinematics kinematics;

    private final Pigeon2 gyro;

    private final double maxSpeedMpS;
    private final double maxRotRadPerSec;

    private PeriodicIO periodicIO = new PeriodicIO();

    private final SwerveDriveOdometry odometry;

    private final Pose2d zeroPose2d = new Pose2d();

    private final double kDt;

    private double pitchZero = 0;

    public static class PeriodicIO {
        // inputs
        public boolean isOpenloop = true;
        public double heading = 0;
        public double roll = 0;
        public double pitch = 0;
        public double rawHeading = 0;
        public Rotation2d gyroRotation = new Rotation2d();

        public SwerveModuleState frontLeftState = new SwerveModuleState();
        public SwerveModuleState frontRightState = new SwerveModuleState();
        public SwerveModuleState backLeftState = new SwerveModuleState();
        public SwerveModuleState backRightState = new SwerveModuleState();

        public SwerveModuleState frontLeftOutputState = new SwerveModuleState();
        public SwerveModuleState frontRightOutputState = new SwerveModuleState();
        public SwerveModuleState backLeftOutputState = new SwerveModuleState();
        public SwerveModuleState backRightOutputState = new SwerveModuleState();

        public double timestamp = 0;
    }

    public SwerveDrive(
            SwerveModule frontLeft,
            SwerveModule frontRight,
            SwerveModule backLeft,
            SwerveModule backRight,
            Pigeon2 gyro,
            SwerveDriveKinematics kinematics,
            double maxSpeedMpS,
            double maxRotRadPerSec,
            double kDt) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
        this.gyro = gyro;
        this.kinematics = kinematics;
        this.maxSpeedMpS = maxSpeedMpS;
        this.maxRotRadPerSec = maxRotRadPerSec;
        this.kDt = kDt;

        this.odometry =
                new SwerveDriveOdometry(
                        this.kinematics,
                        Rotation2d.fromDegrees(gyro.getYaw()),
                        getModulePositions());
    }

    public void zeroPitch() {
        this.pitchZero = this.getPitch();
    }

    @Override
    public void readPeriodicInputs() {
        periodicIO.roll = gyro.getRoll();
        periodicIO.heading = gyro.getYaw();
        periodicIO.pitch = gyro.getPitch() - this.pitchZero;
        periodicIO.rawHeading = gyro.getYaw();
        periodicIO.frontLeftState = frontLeft.getState();
        periodicIO.frontRightState = frontRight.getState();
        periodicIO.backLeftState = backLeft.getState();
        periodicIO.backRightState = backRight.getState();
        periodicIO.gyroRotation = Rotation2d.fromDegrees(periodicIO.heading);
        periodicIO.timestamp = Timer.getFPGATimestamp();

        SmartDashboard.putNumber("yaw", getHeading());
        SmartDashboard.putNumber("pitch", periodicIO.pitch);
        SmartDashboard.putNumber("roll", periodicIO.roll);

        SmartDashboard.putNumber("fl abs", frontLeft.getAbsEncoderPos().getDegrees());
        SmartDashboard.putNumber("fr abs", frontRight.getAbsEncoderPos().getDegrees());
        SmartDashboard.putNumber("bl abs", backLeft.getAbsEncoderPos().getDegrees());
        SmartDashboard.putNumber("br abs", backRight.getAbsEncoderPos().getDegrees());

        odometry.update(Rotation2d.fromDegrees(periodicIO.rawHeading), getModulePositions());
    }

    @Override
    public void writePeriodicOutputs() {
        frontLeft.setDesiredState(periodicIO.frontLeftOutputState, periodicIO.isOpenloop);
        frontRight.setDesiredState(periodicIO.frontRightOutputState, periodicIO.isOpenloop);
        backLeft.setDesiredState(periodicIO.backLeftOutputState, periodicIO.isOpenloop);
        backRight.setDesiredState(periodicIO.backRightOutputState, periodicIO.isOpenloop);
    }

    @Override
    public void periodic() {
        readPeriodicInputs();
        writePeriodicOutputs();
    }

    public void setRobotPose(Pose2d pose) {
        gyro.setYaw(pose.getRotation().getDegrees());
        odometry.resetPosition(pose.getRotation(), getModulePositions(), pose);
        periodicIO = new PeriodicIO();
    }

    public void resetEncoders() {
        frontLeft.resetDriveEncoders();
        frontRight.resetDriveEncoders();
        backLeft.resetDriveEncoders();
        backRight.resetDriveEncoders();
    }

    public void resetModuleAngle() {
        frontLeft.resetToAbsolute();
        frontRight.resetToAbsolute();
        backLeft.resetToAbsolute();
        backRight.resetToAbsolute();
    }

    public double getTimestamp() {
        return periodicIO.timestamp;
    }

    public void zeroGyro() {
        gyro.setYaw(0.0);
    }

    public double getHeading() {
        return periodicIO.heading;
    }

    public double getRoll() {
        return periodicIO.roll;
    }

    public double getPitch() {
        return periodicIO.pitch;
    }

    // Probably want to moving average filter pitch and roll.
    public boolean isBalanced(double thresholdDeg) {
        return Math.abs(getRoll()) < thresholdDeg && Math.abs(getPitch()) < thresholdDeg;
    }

    public double getRawHeading() {
        return periodicIO.rawHeading;
    }

    public Pose2d getOdoPose() {
        return odometry.getPoseMeters();
    }

    public double getPoseX() {
        return odometry.getPoseMeters().getX();
    }

    public double getPoseY() {
        return odometry.getPoseMeters().getY();
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        };
    }

    public void resetOdometry() {
        odometry.resetPosition(
                new Rotation2d(),
                new SwerveModulePosition[] {
                    new SwerveModulePosition(),
                    new SwerveModulePosition(),
                    new SwerveModulePosition(),
                    new SwerveModulePosition()
                },
                new Pose2d());
    }

    @Override
    public void end() {
        stopModules();
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void drive(
            Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = null;
        ChassisSpeeds speeds =
                fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                translation.getX(),
                                translation.getY(),
                                rotation,
                                Rotation2d.fromDegrees(getRawHeading()))
                        : new ChassisSpeeds(translation.getX(), translation.getY(), rotation);

        Pose2d robot_pose_vel =
                new Pose2d(
                        speeds.vxMetersPerSecond * this.kDt,
                        speeds.vyMetersPerSecond * this.kDt,
                        Rotation2d.fromRadians(speeds.omegaRadiansPerSecond * kDt));
        Twist2d twist_vel = robot_pose_vel.log(zeroPose2d);
        ChassisSpeeds updated_chassis_speeds =
                new ChassisSpeeds(twist_vel.dx / kDt, twist_vel.dy / kDt, twist_vel.dtheta / kDt);

        swerveModuleStates = this.kinematics.toSwerveModuleStates(updated_chassis_speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, this.maxSpeedMpS);

        periodicIO.frontLeftOutputState = swerveModuleStates[0];
        periodicIO.frontRightOutputState = swerveModuleStates[1];
        periodicIO.backLeftOutputState = swerveModuleStates[2];
        periodicIO.backRightOutputState = swerveModuleStates[3];

        periodicIO.isOpenloop = isOpenLoop;
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, this.maxSpeedMpS);
        periodicIO.frontLeftOutputState = desiredStates[0];
        periodicIO.frontRightOutputState = desiredStates[1];
        periodicIO.backLeftOutputState = desiredStates[2];
        periodicIO.backRightOutputState = desiredStates[3];
    }

    public void setChasisSpeeds(ChassisSpeeds speeds) {
        SwerveModuleState[] swerveModuleStates = this.kinematics.toSwerveModuleStates(speeds);
        setModuleStates(swerveModuleStates);
    }

    public void setFieldRelativeSpeeds(ChassisSpeeds speeds) {
        var robotSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, periodicIO.gyroRotation);
        var moduleStates = kinematics.toSwerveModuleStates(robotSpeeds);
        setModuleStates(moduleStates);
    }

    public void setModulesAngle(double angle) {
        SwerveModuleState state = new SwerveModuleState(0.1, Rotation2d.fromDegrees(angle));
        SwerveModuleState[] states = {state, state, state, state};
        setModuleStates(states);
    }

    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
            periodicIO.frontLeftState,
            periodicIO.frontRightState,
            periodicIO.backLeftState,
            periodicIO.backRightState
        };
    }

    public double getMaxSpeedMpS() {
        return this.maxSpeedMpS;
    }

    public double getMaxRotationRadpS() {
        return this.maxRotRadPerSec;
    }

    @Override
    public String getName() {
        return "Swerve Drivetrain";
    }
}
