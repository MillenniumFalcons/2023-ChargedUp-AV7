package team3647.frc2023.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import team3647.frc2023.constants.AutoConstants;
import team3647.frc2023.constants.SwerveDriveConstants;
import team3647.frc2023.subsystems.vision.PhotonVisionCamera;
import team3647.lib.GroupPrinter;
import team3647.lib.PeriodicSubsystem;
import team3647.lib.team254.util.MovingAverage;

public class SwerveDrive implements PeriodicSubsystem {
    private final GroupPrinter m_printer =GroupPrinter.getInstance();
    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backLeft;
    private final SwerveModule backRight;

    public final SwerveDrivePoseEstimator poseEstimator;

    private final MovingAverage frontLeftAverageSpeed = new MovingAverage(10);
    private final MovingAverage frontRightAverageSpeed = new MovingAverage(10);
    private final MovingAverage backLeftAverageSpeed = new MovingAverage(10);
    private final MovingAverage backRightAverageSpeed = new MovingAverage(10);

    private final Pigeon2 gyro;

    private PeriodicIO periodicIO = new PeriodicIO();

    private final SwerveDriveOdometry odometry;
    public static class PeriodicIO {
        // inputs
        public double timestamp = 0;
        public boolean isOpenLoop = true;
        public double heading = 0;
        public double rawHeading = 0;
        public boolean solenoidState = false;

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
            Pigeon2 gyro) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
        this.gyro = gyro;
        this.poseEstimator = new SwerveDrivePoseEstimator(SwerveDriveConstants.kDriveKinematics, getRotation2d(), getSwerveModulePositions(),new Pose2d());
        this.odometry =
                new SwerveDriveOdometry(
                        SwerveDriveConstants.kDriveKinematics,
                        Rotation2d.fromDegrees(periodicIO.heading),
                        new SwerveModulePosition[]{
                        frontLeft.getPosition(),
                        frontRight.getPosition(),
                        backLeft.getPosition(),
                        backRight.getPosition()
                        });
    }

    @Override
    public void init() {
        System.out.println("init-ed swerve bruh i wanna die");
        resetEncoders();
        // resetOdometry();
        zeroHeading();
    }

    @Override
    public void readPeriodicInputs() {
        // periodicIO.heading = Math.IEEEremainder(gyro.getYaw(), 360);
        periodicIO.heading = gyro.getYaw();
        periodicIO.rawHeading = gyro.getYaw();
        periodicIO.frontLeftState = frontLeft.getState();
        periodicIO.frontRightState = frontRight.getState();
        periodicIO.backLeftState = backLeft.getState();
        periodicIO.backRightState = backRight.getState();

        frontLeftAverageSpeed.add(periodicIO.frontLeftState.speedMetersPerSecond);
        frontRightAverageSpeed.add(periodicIO.frontRightState.speedMetersPerSecond);
        backLeftAverageSpeed.add(periodicIO.backLeftState.speedMetersPerSecond);
        backRightAverageSpeed.add(periodicIO.backRightState.speedMetersPerSecond);

        // SmartDashboard.putNumber("ABS FL", frontLeft.getAbsEncoderPos().getDegrees());
        // SmartDashboard.putNumber("ABS FR", frontRight.getAbsEncoderPos().getDegrees());
        // SmartDashboard.putNumber("ABS BL", backLeft.getAbsEncoderPos().getDegrees());
        // SmartDashboard.putNumber("ABS BR", backRight.getAbsEncoderPos().getDegrees());

        // SmartDashboard.putNumber("FL", frontLeft.getTurnAngle());
        // SmartDashboard.putNumber("FR", frontRight.getTurnAngle());
        // SmartDashboard.putNumber("BL", backLeft.getTurnAngle());
        // SmartDashboard.putNumber("BR", backRight.getTurnAngle());

        SmartDashboard.putNumber("FL speed", periodicIO.frontLeftState.speedMetersPerSecond);
        SmartDashboard.putNumber("FR speed", periodicIO.frontRightState.speedMetersPerSecond);
        SmartDashboard.putNumber("BL speed", periodicIO.backLeftState.speedMetersPerSecond);
        SmartDashboard.putNumber("BR speed", periodicIO.backRightState.speedMetersPerSecond);

        SmartDashboard.putNumber("FL angle", periodicIO.frontLeftState.angle.getDegrees());
        SmartDashboard.putNumber("FR angle", periodicIO.frontRightState.angle.getDegrees());
        SmartDashboard.putNumber("BL angle", periodicIO.backLeftState.angle.getDegrees());
        SmartDashboard.putNumber("BR angle", periodicIO.backRightState.angle.getDegrees());

        odometry.update(
                getRotation2d(),
                new SwerveModulePosition[]{
                    frontLeft.getPosition(),
                    frontRight.getPosition(),
                    backLeft.getPosition(),
                    backRight.getPosition()
                });

        // update pose estimator
        poseEstimator.update(getRotation2d(), getSwerveModulePositions());
        periodicIO.timestamp = Timer.getFPGATimestamp();
    //     Pair<Pose2d, Double> result = camera.getEstimatedGlobalPose(poseEstimator.getEstimatedPosition());
    //     var camPose = result.getFirst();
    //     var camPoseObsTime = result.getSecond();
    //     if (camPose != null) {
    //         poseEstimator.addVisionMeasurement(camPose, camPoseObsTime);
    //     }

    //     m_printer.addPose("robot pose", this::getPose);
    }

    @Override
    public void writePeriodicOutputs() {
        // solenoid.set(periodicIO.solenoidState);
        frontLeft.setDesiredState(periodicIO.frontLeftOutputState, periodicIO.isOpenLoop);
        frontRight.setDesiredState(periodicIO.frontRightOutputState, periodicIO.isOpenLoop);
        backLeft.setDesiredState(periodicIO.backLeftOutputState, periodicIO.isOpenLoop);
        backRight.setDesiredState(periodicIO.backRightOutputState, periodicIO.isOpenLoop);
    }

    public void extend() {
        periodicIO.solenoidState = true;
    }

    public void retract() {
        periodicIO.solenoidState = false;
    }

    @Override
    public void periodic() {
        readPeriodicInputs();
        writePeriodicOutputs();
    }

    public void setOdometry(Pose2d pose, Rotation2d rot) {
        SmartDashboard.putNumber("rot", rot.getDegrees());
        odometry.resetPosition(rot,
        new SwerveModulePosition[]{
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()},pose);
        gyro.setYaw(pose.getRotation().getDegrees());

        poseEstimator.resetPosition(rot, new SwerveModulePosition[]{
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()}, pose);
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
        gyro.setYaw(0);
    }

    public double getHeading() {
        return periodicIO.heading;
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

    public double getEstimX() {
        return poseEstimator.getEstimatedPosition().getX();
    }

    public double getEstimY() {
        return poseEstimator.getEstimatedPosition().getY();
    }

    public SwerveModulePosition[] getSwerveModulePositions() {
        return new SwerveModulePosition[]{
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        };
    }

    public void resetOdometry() {
        odometry.resetPosition(new Rotation2d(), new SwerveModulePosition[]{
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
        }, new Pose2d());
    }

    public double getTimestamp() {
        return periodicIO.timestamp;
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

        swerveModuleStates =
                SwerveDriveConstants.kDriveKinematics.toSwerveModuleStates(
                        fieldRelative
                                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                        translation.getX(),
                                        translation.getY(),
                                        rotation,
                                        getRotation2d())
                                : new ChassisSpeeds(
                                        translation.getX(), translation.getY(), rotation));
        SwerveDriveKinematics.desaturateWheelSpeeds(
                swerveModuleStates, SwerveDriveConstants.kDrivePossibleMaxSpeedMPS);

        periodicIO.frontLeftOutputState = swerveModuleStates[0];
        periodicIO.frontRightOutputState = swerveModuleStates[1];
        periodicIO.backLeftOutputState = swerveModuleStates[2];
        periodicIO.backRightOutputState = swerveModuleStates[3];

        periodicIO.isOpenLoop = isOpenLoop;
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
                desiredStates, SwerveDriveConstants.kDrivePossibleMaxSpeedMPS);
        periodicIO.frontLeftOutputState = desiredStates[0];
        periodicIO.frontRightOutputState = desiredStates[1];
        periodicIO.backLeftOutputState = desiredStates[2];
        periodicIO.backRightOutputState = desiredStates[3];
    }

    public void setChasisSpeeds(ChassisSpeeds speeds) {
        SwerveModuleState[] swerveModuleStates = SwerveDriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
        setModuleStates(swerveModuleStates);
    }

    public void setModulesAngle(double angle) {
        SwerveModuleState state = new SwerveModuleState(0.1, Rotation2d.fromDegrees(angle));
        SwerveModuleState[] states = {state, state, state, state};
        setModuleStates(states);
    }

    public PathPlannerTrajectory getToPointATrajectory(PathPoint endpoint) {
        return PathPlanner.generatePath(new PathConstraints(1,1), PathPoint.fromCurrentHolonomicState(getPose(), 
        SwerveDriveConstants.kDriveKinematics.toChassisSpeeds(periodicIO.frontLeftState, periodicIO.frontRightState, periodicIO.backLeftState, periodicIO.backRightState)),
        endpoint);
    }

    public PPSwerveControllerCommand getTrajectoryCommand(PathPlannerTrajectory trajectory) {
        return new PPSwerveControllerCommand(trajectory, this::getPose, AutoConstants.kXController, AutoConstants.kYController, AutoConstants.kRotController, this::setChasisSpeeds, this);
    }    

    @Override
    public String getName() {
        // TODO Auto-generated method stub
        return "Swerve Drivetrain";
    }
}
