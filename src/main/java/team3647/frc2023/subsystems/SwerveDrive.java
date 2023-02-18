package team3647.frc2023.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import team3647.frc2023.constants.AutoConstants;
import team3647.frc2023.constants.SwerveDriveConstants;
import team3647.lib.PeriodicSubsystem;
import team3647.lib.SwerveModule;

public class SwerveDrive implements PeriodicSubsystem {
    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backLeft;
    private final SwerveModule backRight;

    public final SwerveDrivePoseEstimator poseEstimator;
    private final SwerveDriveKinematics kinematics;

    private final Pigeon2 gyro;

    private final double maxSpeedMpS;
    private final double maxRotRadPerSec;

    private PeriodicIO periodicIO = new PeriodicIO();

    private final SwerveDriveOdometry odometry;

    public static class PeriodicIO {
        // inputs
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
                new SwerveDriveOdometry(this.kinematics, getRotation2d(), getModulePositions());
        zeroGyro();
    }

    @Override
    public void init() {
        resetEncoders();
        zeroGyro();
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

        SmartDashboard.putNumber("yaw", getHeading());
        SmartDashboard.putNumber("pitch", periodicIO.pitch);
        SmartDashboard.putNumber("roll", periodicIO.roll);
        SmartDashboard.putNumber("robot pose rot", getEstimPose().getRotation().getDegrees());

        SmartDashboard.putNumber("fl abs", frontLeft.getAbsEncoderPos().getDegrees());
        SmartDashboard.putNumber("fr abs", frontRight.getAbsEncoderPos().getDegrees());
        SmartDashboard.putNumber("bl abs", backLeft.getAbsEncoderPos().getDegrees());
        SmartDashboard.putNumber("br abs", backRight.getAbsEncoderPos().getDegrees());

        // SmartDashboard.putNumber("FL angle", periodicIO.frontLeftState.angle.getDegrees());
        // SmartDashboard.putNumber("FR angle", periodicIO.frontRightState.angle.getDegrees());
        // SmartDashboard.putNumber("BL angle", periodicIO.backLeftState.angle.getDegrees());
        // SmartDashboard.putNumber("BR angle", periodicIO.backRightState.angle.getDegrees());

        // SmartDashboard.putNumber(
        //         "fl diff",
        //         frontLeft.getAbsEncoderPos().getDegrees()
        //                 - SwerveDriveConstants.kAbsFrontLeftEncoderOffsetDeg);
        // SmartDashboard.putNumber(
        //         "fr diff",
        //         frontRight.getAbsEncoderPos().getDegrees()
        //                 - SwerveDriveConstants.kAbsFrontRightEncoderOffsetDeg);
        // SmartDashboard.putNumber(
        //         "bl diff",
        //         backLeft.getAbsEncoderPos().getDegrees()
        //                 - SwerveDriveConstants.kAbsBackLeftEncoderOffsetDeg);
        // SmartDashboard.putNumber(
        //         "br diff",
        //         backRight.getAbsEncoderPos().getDegrees()
        //                 - SwerveDriveConstants.kAbsBackRightEncoderOffsetDeg);

        // SmartDashboard.putNumber(getName(), maxRotRadPerSec)

        odometry.update(getRotation2d(), getModulePositions());
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

    public void setRobotPose(Pose2d pose) {
        odometry.resetPosition(getRotation2d(), getModulePositions(), pose);
        poseEstimator.resetPosition(getRotation2d(), getModulePositions(), pose);
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

    public void addVisionMeasurement(Double timestamp, Pose2d visionBotPose2d) {
        if (timestamp == null || visionBotPose2d == null) {
            return;
        }

        if (poseEstimator
                        .getEstimatedPosition()
                        .getTranslation()
                        .minus(visionBotPose2d.getTranslation())
                        .getNorm()
                > 1) return;
        Pose2d acutalPose2d = new Pose2d(visionBotPose2d.getTranslation(), this.getRotation2d());
        // GroupPrinter.getInstance().getField().getObject("vision pose").setPose(acutalPose2d);
        this.poseEstimator.addVisionMeasurement(acutalPose2d, timestamp.doubleValue());
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

    public PathPlannerTrajectory getToPointATrajectory(PathPoint endpoint) {
        return PathPlanner.generatePath(
                new PathConstraints(1, 1),
                PathPoint.fromCurrentHolonomicState(
                        getEstimPose(),
                        SwerveDriveConstants.kDriveKinematics.toChassisSpeeds(
                                periodicIO.frontLeftState,
                                periodicIO.frontRightState,
                                periodicIO.backLeftState,
                                periodicIO.backRightState)),
                endpoint);
    }

    public PPSwerveControllerCommand getTrajectoryCommand(PathPlannerTrajectory trajectory) {
        return new PPSwerveControllerCommand(
                trajectory,
                this::getEstimPose,
                AutoConstants.kXController,
                AutoConstants.kYController,
                AutoConstants.kRotController,
                this::setChasisSpeeds,
                this);
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
