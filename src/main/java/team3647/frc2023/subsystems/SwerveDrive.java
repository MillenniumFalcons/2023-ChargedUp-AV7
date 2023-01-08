package team3647.frc2023.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
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
import team3647.lib.team254.util.MovingAverage;

public class SwerveDrive implements PeriodicSubsystem {
    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backLeft;
    private final SwerveModule backRight;

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
        resetOdometry();
        zeroHeading();
    }

    @Override
    public void readPeriodicInputs() {
        periodicIO.heading = Math.IEEEremainder(gyro.getYaw(), 360);
        periodicIO.rawHeading = gyro.getYaw();
        periodicIO.frontLeftState = frontLeft.getState();
        periodicIO.frontRightState = frontRight.getState();
        periodicIO.backLeftState = backLeft.getState();
        periodicIO.backRightState = backRight.getState();

        frontLeftAverageSpeed.add(periodicIO.frontLeftState.speedMetersPerSecond);
        frontRightAverageSpeed.add(periodicIO.frontRightState.speedMetersPerSecond);
        backLeftAverageSpeed.add(periodicIO.backLeftState.speedMetersPerSecond);
        backRightAverageSpeed.add(periodicIO.backRightState.speedMetersPerSecond);

        odometry.update(
                getRotation2d(),
                new SwerveModulePosition[]{
                    frontLeft.getPosition(),
                    frontRight.getPosition(),
                    backLeft.getPosition(),
                    backRight.getPosition()
                });

        periodicIO.timestamp = Timer.getFPGATimestamp();
    }

    @Override
    public void writePeriodicOutputs() {

        frontLeft.setDesiredState(periodicIO.frontLeftOutputState, periodicIO.isOpenLoop);
        frontRight.setDesiredState(periodicIO.frontRightOutputState, periodicIO.isOpenLoop);
        backLeft.setDesiredState(periodicIO.backLeftOutputState, periodicIO.isOpenLoop);
        backRight.setDesiredState(periodicIO.backRightOutputState, periodicIO.isOpenLoop);

        // SmartDashboard.putNumber(
        //         "Swerve Turn Demand", periodicIO.backLeftOutputState.angle.getDegrees());

        SmartDashboard.putNumber("Robot Rotation", getHeading());
        SmartDashboard.putNumber(
                "FL ABS", frontLeft.getAbsEncoderPos().getDegrees() - frontLeft.absOffsetDeg);
        SmartDashboard.putNumber(
                "FR ABS", frontRight.getAbsEncoderPos().getDegrees() - frontRight.absOffsetDeg);
        SmartDashboard.putNumber(
                "BL ABS", backLeft.getAbsEncoderPos().getDegrees() - backLeft.absOffsetDeg);
        SmartDashboard.putNumber(
                "BR ABS", backRight.getAbsEncoderPos().getDegrees() - backRight.absOffsetDeg);

        SmartDashboard.putNumber("FL INT", frontLeft.getTurnAngle());
        SmartDashboard.putNumber("FR INT", frontRight.getTurnAngle());
        SmartDashboard.putNumber("BL INT", backLeft.getTurnAngle());
        SmartDashboard.putNumber("BR INT", backRight.getTurnAngle());
        SmartDashboard.putString("Robot XY", getPose().getTranslation().toString());

        SmartDashboard.putNumber("FL speed real", frontLeft.getDriveVelocity());
        SmartDashboard.putNumber(
                "FL speed demand", periodicIO.frontLeftOutputState.speedMetersPerSecond);

        SmartDashboard.putNumber("FR speed real", frontRight.getDriveVelocity());
        SmartDashboard.putNumber(
                "FR speed demand", periodicIO.frontRightOutputState.speedMetersPerSecond);

        SmartDashboard.putNumber("BL speed real", backLeft.getDriveVelocity());
        SmartDashboard.putNumber(
                "BL speed demand", periodicIO.backLeftOutputState.speedMetersPerSecond);

        SmartDashboard.putNumber("BR speed real", backRight.getDriveVelocity());
        SmartDashboard.putNumber(
                "BR speed demand", periodicIO.backRightOutputState.speedMetersPerSecond);

        SmartDashboard.putNumber("FL Output Voltage", frontLeft.getVoltage());

        SmartDashboard.putNumber("FR Output Voltage", frontRight.getVoltage());

        SmartDashboard.putNumber("BL Output Voltage", backLeft.getVoltage());

        SmartDashboard.putNumber("BR Output Voltage", backRight.getVoltage());

        // SmartDashboard.putNumber(
        //         "front left, ABS Angle", frontLeft.getAbsEncoderPos().getDegrees());
        // SmartDashboard.putNumber("front left, INT Angle", frontLeft.getTurnAngle());
        // SmartDashboard.putNumber("front left, Drive Velocity", frontLeft.getDriveVelocity());
        // SmartDashboard.putNumber(
        //         "front left diff",
        //         frontLeft.getDriveVelocity()
        //                 - periodicIO.frontLeftOutputState.speedMetersPerSecond);

        // SmartDashboard.putNumber(
        //         "front right, ABS Angle", frontRight.getAbsEncoderPos().getDegrees());
        // SmartDashboard.putNumber("front right, INT Angle", frontRight.getTurnAngle());
        // SmartDashboard.putNumber("front right, Drive Velocity", frontRight.getDriveVelocity());
        // SmartDashboard.putNumber(
        //         "front right diff",
        //         frontRight.getDriveVelocity()
        //                 - periodicIO.frontRightOutputState.speedMetersPerSecond);

        // SmartDashboard.putNumber("back left, ABS Angle",
        // backLeft.getAbsEncoderPos().getDegrees());
        // SmartDashboard.putNumber("back left, INT Angle", backLeft.getTurnAngle());
        // SmartDashboard.putNumber("back left, Drive Velocity", backLeft.getDriveVelocity());
        // SmartDashboard.putNumber(
        //         "back left diff",
        //         backLeft.getDriveVelocity() -
        // periodicIO.backLeftOutputState.speedMetersPerSecond);

        // SmartDashboard.putNumber(
        //         "back right, ABS Angle", backRight.getAbsEncoderPos().getDegrees());
        // SmartDashboard.putNumber("back right, INT Angle", backRight.getTurnAngle());
        // SmartDashboard.putNumber("back right, Drive Velocity", backRight.getDriveVelocity());
        // SmartDashboard.putNumber(
        //         "back right diff",
        //         backRight.getDriveVelocity()
        //                 - periodicIO.backRightOutputState.speedMetersPerSecond);

        // SmartDashboard.putString("Mod 0 demand State",
        // periodicIO.frontRightOutputState.toString());
        // SmartDashboard.putString("Mod 1 demand State",
        // periodicIO.frontLeftOutputState.toString());
        // SmartDashboard.putString("Mod 2 demand State",
        // periodicIO.backRightOutputState.toString());
        // SmartDashboard.putString("Mod 3 demand State",
        // periodicIO.backLeftOutputState.toString());

        // SmartDashboard.putNumber("Percent Out 0", frontRight.percentOut);
        // SmartDashboard.putNumber("Percent Out 1", frontLeft.percentOut);
        // SmartDashboard.putNumber("Percent Out 2", backRight.percentOut);
        // SmartDashboard.putNumber("Percent Out 3", backLeft.percentOut);
    }

    @Override
    public void periodic() {
        // readPeriodicInputs();
        writePeriodicOutputs();
    }

    // public void setAbsoluteZeros() {
    //     System.out.println(
    //             "front left Setting Zero "
    //                     + frontLeft.absEncoder.configGetMagnetOffset()
    //                     + " -> 0");
    //     frontLeft.absEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
    //     frontLeft.absEncoder.configMagnetOffset(
    //             -(frontLeft.absEncoder.getAbsolutePosition()
    //                     - frontLeft.absEncoder.configGetMagnetOffset()));

    //     System.out.println(
    //             "front right Setting Zero "
    //                     + frontRight.absEncoder.configGetMagnetOffset()
    //                     + " -> 0");
    //     frontRight.absEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
    //     frontRight.absEncoder.configMagnetOffset(
    //             -(frontRight.absEncoder.getAbsolutePosition()
    //                     - frontRight.absEncoder.configGetMagnetOffset()));

    //     System.out.println(
    //             "back left Setting Zero " + backLeft.absEncoder.configGetMagnetOffset() + " ->
    // 0");
    //     backLeft.absEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
    //     backLeft.absEncoder.configMagnetOffset(
    //             -(backLeft.absEncoder.getAbsolutePosition()
    //                     - backLeft.absEncoder.configGetMagnetOffset()));

    //     System.out.println(
    //             "back right Setting Zero "
    //                     + backRight.absEncoder.configGetMagnetOffset()
    //                     + " -> 0");
    //     backRight.absEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
    //     backRight.absEncoder.configMagnetOffset(
    //             -(backRight.absEncoder.getAbsolutePosition()
    //                     - backRight.absEncoder.configGetMagnetOffset()));
    // }

    public void setOdometry(Pose2d pose, Rotation2d swerveHeading) {
        odometry.resetPosition(pose.getRotation(),
        new SwerveModulePosition[]{
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()},pose);
        gyro.setYaw(swerveHeading.getDegrees());

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

    // public void setAbsoluteZeros() {
    //     System.out.println(i + " Setting Zero " + swerveCanCoder.configGetMagnetOffset() + " ->
    // 0");
    //     frontLeft..configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
    //     swerveCanCoder.configMagnetOffset(
    //             -(swerveCanCoder.getAbsolutePosition() -
    // swerveCanCoder.configGetMagnetOffset()));
    // }

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

    public void setModulesAngle(double angle) {
        SwerveModuleState state = new SwerveModuleState(0.1, Rotation2d.fromDegrees(angle));
        SwerveModuleState[] states = {state, state, state, state};
        setModuleStates(states);
    }

    // public boolean isStopped(double threshold) {
    //     return Math.abs(rightAverageSpeed.getAverage()) < threshold
    //             && Math.abs(leftAverageSpeed.getAverage()) < threshold;
    // }

    public boolean isStopped(double threshold) {
        // return Math.abs(frontLeftAverageSpeed.getAverage()) < threshold
        //         && Math.abs(frontRightAverageSpeed.getAverage()) < threshold
        //         && Math.abs(backLeftAverageSpeed.getAverage()) < threshold
        //         && Math.abs(backRightAverageSpeed.getAverage()) < threshold;
        return true;
    }

    public boolean isStopped() {
        return isStopped(0.0127);
    }

    @Override
    public String getName() {
        // TODO Auto-generated method stub
        return "Swerve Drivetrain";
    }
}
