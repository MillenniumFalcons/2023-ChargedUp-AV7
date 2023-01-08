package team3647.frc2023.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import team3647.frc2023.constants.SwerveDriveConstants;
import team3647.lib.CTREModuleState;

public class SwerveModule {
    private final TalonFX driveMotor;
    private final TalonFX turnMotor;

    private final SimpleMotorFeedforward ff;
    // abs since motor encoder resets
    public final CANCoder absEncoder;
    // offset reading of motor and abs pos after reset
    public final double absOffsetDeg;

    private final double driveVelocityConversion;
    private final double drivePositionConversion;
    private final double turnVelocityConversion;
    private final double turnPositionConversion;


    private double lastAngle;

    public double percentOut = 0;

    public SwerveModule(
            TalonFX driveMotor,
            TalonFX turnMotor,
            SimpleMotorFeedforward ff,
            CANCoder absEncoder,
            double absOffsetDeg,
            double kDriveVelocityConversion,
            double kDrivePositionConversion,
            double kTurnVelocityConversion,
            double kTurnPositionConversion,
            double nominalVoltage) {
        this.driveMotor = driveMotor;
        this.turnMotor = turnMotor;
        this.ff = ff;
        this.absEncoder = absEncoder;
        this.absOffsetDeg = absOffsetDeg;
        this.driveVelocityConversion = kDriveVelocityConversion;
        this.drivePositionConversion = kDrivePositionConversion;
        this.turnVelocityConversion = kTurnVelocityConversion;
        this.turnPositionConversion = kTurnPositionConversion;
        this.lastAngle = getState().angle.getDegrees();
        setNeutralMode(
                SwerveDriveConstants.kTurnNeutralMode, SwerveDriveConstants.kDriveNeutralMode);
        resetToAbsolute();
    }

    public double getDrivePos() {
        return driveMotor.getSelectedSensorPosition() * drivePositionConversion;
    }

    public double getTurnAngle() {
        return turnMotor.getSelectedSensorPosition() * turnPositionConversion;
    }

    public double getDriveVelocity() {
        return driveMotor.getSelectedSensorVelocity() * driveVelocityConversion;
    }

    public double getTurnVelocity() {
        return turnMotor.getSelectedSensorVelocity() * turnVelocityConversion;
    }

    public Rotation2d getAbsEncoderPos() {
        return Rotation2d.fromDegrees(absEncoder.getAbsolutePosition());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
                getDriveVelocity(), new Rotation2d(Units.degreesToRadians(getTurnAngle())));
    }
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            getDrivePos(), Rotation2d.fromDegrees((getTurnAngle())));
    }

    public double getVoltage() {
        return driveMotor.getMotorOutputVoltage();
    }

    public void resetToAbsolute() {
        double absolutePosition =
                (getCanCoder().getDegrees() - absOffsetDeg) / turnPositionConversion;
        turnMotor.setSelectedSensorPosition(absolutePosition);

        // double absoluteAngle = (getAbsEncoderPos().getDegrees() - absOffsetDeg);
        // double adjustedAngle = CTREModuleState.optimizeAngle(absoluteAngle, getTurnAngle());
        // turnMotor.setSelectedSensorPosition(adjustedAngle / turnPositionConversion);
    }

    public Rotation2d getCanCoder() {
        return Rotation2d.fromDegrees(absEncoder.getAbsolutePosition());
    }

    public CANCoder getCanCoderObject() {
        return this.absEncoder;
    }

    public void resetDriveEncoders() {
        driveMotor.setSelectedSensorPosition(0);
    }

    public void setNeutralMode(NeutralMode turnNeutralMode, NeutralMode driveNeutralMode) {
        turnMotor.setNeutralMode(turnNeutralMode);
        driveMotor.setNeutralMode(driveNeutralMode);
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {

        desiredState =
                CTREModuleState.optimize(
                        desiredState,
                        getState().angle); // Custom optimize command, since default WPILib optimize
        // assumes continuous controller which CTRE is not

        if (isOpenLoop) {
            double percentOutput =
                    desiredState.speedMetersPerSecond
                            / SwerveDriveConstants.kDrivePossibleMaxSpeedMPS;
            driveMotor.set(ControlMode.PercentOutput, percentOutput);
            this.percentOut = percentOutput;
        } else {
            // MPS to falcon
            double velocity = desiredState.speedMetersPerSecond / driveVelocityConversion;
            driveMotor.set(
                    ControlMode.Velocity,
                    velocity,
                    DemandType.ArbitraryFeedForward,
                    ff.calculate(desiredState.speedMetersPerSecond));
        }

        double angle =
                (Math.abs(desiredState.speedMetersPerSecond)
                                <= (SwerveDriveConstants.kDrivePossibleMaxSpeedMPS * 0.01))
                        ? lastAngle
                        : desiredState.angle.getDegrees();
        // Prevents Jittering.
        turnMotor.set(ControlMode.Position, angle / turnPositionConversion);
        lastAngle = angle;
    }

    public void stop() {
        // stop velocity, not set position to 0
        driveMotor.set(ControlMode.PercentOutput, 0);
        turnMotor.set(ControlMode.PercentOutput, 0);
    }
}
