package team3647.frc2023.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import team3647.frc2023.constants.SwerveDriveConstants;
import team3647.lib.PeriodicSubsystem;
import team3647.lib.SwerveModule;
import team3647.lib.team254.util.MovingAverage;

public class SwerveDrive implements PeriodicSubsystem {
    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backLeft;
    private final SwerveModule backRight;

    public final SwerveDrivePoseEstimator poseEstimator;

    private final MovingAverage frontLeftAverageSpeed = new MovingAverage(10);
    private final MovingAverage frontRightAverageSpeed = new MovingAverage(10);
    private final MovingAverage backLeftAverageSpeed = new MovingAverage(10);
    private final MovingAverage backRightAverageSpeed = new MovingAverage(10);

    private final SwerveDriveKinematics kinematics;

    private final Pigeon2 gyro;

    private final double maxSpeedMpS;
    private final double maxRotRadPerSec;

    private PeriodicIO periodicIO = new PeriodicIO();

    private final SwerveDriveOdometry odometry;

    public static class PeriodicIO {
        // inputs
        public double timestamp = 0;
        public boolean isOpenLoop = true;
        public double heading = 0;
        public double roll = 0;
        public double pitch = 0;
        public double rawHeading = 0;

        public SwerveModuleState frontLeftState = new SwerveModuleState();
        public SwerveModuleState frontRightState = new SwerveModuleState();
        public SwerveModuleState backLeftState = new SwerveModuleState();
        public SwerveModuleState backRightState = new SwerveModuleState();

        public SwerveModuleState frontLeftOutputState = new SwerveModuleState();
        public SwerveModuleState frontRightOutputState = new SwerveModuleState();
        public SwerveModuleState backLeftOutputState = new SwerveModuleState();
        public SwerveModuleState backRightOutputState = new SwerveModuleState();
    }

    public SwerveDrive(
            SwerveModule frontLeft,
            SwerveModule frontRight,
            SwerveModule backLeft,
            SwerveModule backRight,
            Pigeon2 gyro,
            SwerveDriveKinematics kinematics,
            double maxSpeedMpS,
            double maxRotRadPerSec) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
        this.gyro = gyro;
        this.kinematics = kinematics;
        this.maxSpeedMpS = maxSpeedMpS;
        this.maxRotRadPerSec = maxRotRadPerSec;
        this.poseEstimator =
                new SwerveDrivePoseEstimator(
                        this.kinematics, getRotation2d(), getModulePositions(), new Pose2d());
        this.odometry =
                new SwerveDriveOdometry(
                        this.kinematics,
                        Rotation2d.fromDegrees(periodicIO.heading),
                        getModulePositions());
    }

    @Override
    public void init() {
        resetEncoders();
        zeroHeading();
    }

    @Override
    public void readPeriodicInputs() {
        periodicIO.roll = gyro.getRoll();
        periodicIO.heading = gyro.getYaw();
        periodicIO.pitch = gyro.getPitch();
        periodicIO.rawHeading = gyro.getYaw();
        periodicIO.frontLeftState = frontLeft.getState();
        periodicIO.frontRightState = frontRight.getState();
        periodicIO.backLeftState = backLeft.getState();
        periodicIO.backRightState = backRight.getState();

        frontLeftAverageSpeed.add(periodicIO.frontLeftState.speedMetersPerSecond);
        frontRightAverageSpeed.add(periodicIO.frontRightState.speedMetersPerSecond);
        backLeftAverageSpeed.add(periodicIO.backLeftState.speedMetersPerSecond);
        backRightAverageSpeed.add(periodicIO.backRightState.speedMetersPerSecond);

        SmartDashboard.putNumber("fl abs", frontLeft.getAbsEncoderPos().getDegrees());
        SmartDashboard.putNumber("fr abs", frontRight.getAbsEncoderPos().getDegrees());
        SmartDashboard.putNumber("bl abs", backLeft.getAbsEncoderPos().getDegrees());
        SmartDashboard.putNumber("br abs", backRight.getAbsEncoderPos().getDegrees());

        SmartDashboard.putNumber("FL angle", periodicIO.frontLeftState.angle.getDegrees());
        SmartDashboard.putNumber("FR angle", periodicIO.frontRightState.angle.getDegrees());
        SmartDashboard.putNumber("BL angle", periodicIO.backLeftState.angle.getDegrees());
        SmartDashboard.putNumber("BR angle", periodicIO.backRightState.angle.getDegrees());

        SmartDashboard.putNumber(
                "fl diff",
                frontLeft.getAbsEncoderPos().getDegrees()
                        - SwerveDriveConstants.kAbsFrontLeftEncoderOffsetDeg);
        SmartDashboard.putNumber(
                "fr diff",
                frontRight.getAbsEncoderPos().getDegrees()
                        - SwerveDriveConstants.kAbsFrontRightEncoderOffsetDeg);
        SmartDashboard.putNumber(
                "bl diff",
                backLeft.getAbsEncoderPos().getDegrees()
                        - SwerveDriveConstants.kAbsBackLeftEncoderOffsetDeg);
        SmartDashboard.putNumber(
                "br diff",
                backRight.getAbsEncoderPos().getDegrees()
                        - SwerveDriveConstants.kAbsBackRightEncoderOffsetDeg);

        odometry.update(getRotation2d(), getModulePositions());

        // update pose estimator
        periodicIO.timestamp = Timer.getFPGATimestamp();
        poseEstimator.update(getRotation2d(), getModulePositions());
    }

    @Override
    public void writePeriodicOutputs() {
        frontLeft.setDesiredState(periodicIO.frontLeftOutputState, periodicIO.isOpenLoop);
        frontRight.setDesiredState(periodicIO.frontRightOutputState, periodicIO.isOpenLoop);
        backLeft.setDesiredState(periodicIO.backLeftOutputState, periodicIO.isOpenLoop);
        backRight.setDesiredState(periodicIO.backRightOutputState, periodicIO.isOpenLoop);
    }

    @Override
    public void periodic() {
        readPeriodicInputs();
        writePeriodicOutputs();
    }

    public void setOdometry(Pose2d pose, Rotation2d rot) {
        SmartDashboard.putNumber("rot", rot.getDegrees());
        odometry.resetPosition(rot, getModulePositions(), pose);
        gyro.setYaw(pose.getRotation().getDegrees());

        poseEstimator.resetPosition(rot, getModulePositions(), pose);
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

    public void zeroHeading() {
        var pose = getPose();
        var newPose = new Pose2d(pose.getTranslation(), new Rotation2d());
        setOdometry(newPose, new Rotation2d(0));
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

    public void addVisionMeasurment(Double timestamp, Pose2d robotPose) {
        if (timestamp == null || robotPose == null) {
            return;
        }
        this.poseEstimator.addVisionMeasurement(robotPose, timestamp);
    }

    // Probably want to moving average filter pitch and roll.
    public boolean isBalanced(double thresholdDeg) {
        return Math.abs(getRoll()) < thresholdDeg && Math.abs(getPitch()) < thresholdDeg;
    }

    public double getRawHeading() {
        return periodicIO.rawHeading;
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public double getPoseX() {
        return odometry.getPoseMeters().getX();
    }

    public double getPoseY() {
        return odometry.getPoseMeters().getY();
    }

    public Pose2d getEstimPose() {
        return poseEstimator.getEstimatedPosition();
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

    public double getTimestamp() {
        return periodicIO.timestamp;
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

    public void spin() {
        SwerveModuleState[] swerveModuleStates =
                this.kinematics.toSwerveModuleStates(
                        ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, getRotation2d()));

        periodicIO.frontLeftOutputState = swerveModuleStates[0];
        periodicIO.frontRightOutputState = swerveModuleStates[1];
        periodicIO.backLeftOutputState = swerveModuleStates[2];
        periodicIO.backRightOutputState = swerveModuleStates[3];

        periodicIO.isOpenLoop = false;
    }

    public void drive(
            Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = null;

        swerveModuleStates =
                this.kinematics.toSwerveModuleStates(
                        fieldRelative
                                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                        translation.getX(),
                                        translation.getY(),
                                        rotation,
                                        getRotation2d())
                                : new ChassisSpeeds(
                                        translation.getX(), translation.getY(), rotation));
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, this.maxSpeedMpS);

        periodicIO.frontLeftOutputState = swerveModuleStates[0];
        periodicIO.frontRightOutputState = swerveModuleStates[1];
        periodicIO.backLeftOutputState = swerveModuleStates[2];
        periodicIO.backRightOutputState = swerveModuleStates[3];

        periodicIO.isOpenLoop = isOpenLoop;
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
