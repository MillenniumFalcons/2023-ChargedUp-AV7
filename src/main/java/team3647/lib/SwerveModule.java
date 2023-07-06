package team3647.lib;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import team3647.frc2023.constants.SwerveDriveConstants;

public class SwerveModule {
    private final TalonFX driveMotor;
    private final TalonFX turnMotor;

    private final SimpleMotorFeedforward ff;
    // abs since motor encoder resets
    public final CANcoder absEncoder;
    // offset reading of motor and abs pos after reset
    public final double absOffsetDeg;

    private final double driveVelocityConversion;
    private final double drivePositionConversion;
    private final double turnVelocityConversion;
    private final double turnPositionConversion;

    private final double nominalVoltage;

    private double lastAngle;

    public double percentOut = 0;

    private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0);
    private final VelocityDutyCycle velocityDutyCycle = new VelocityDutyCycle(0);
    private final PositionDutyCycle positionDutyCycle = new PositionDutyCycle(0);

    public SwerveModule(
            TalonFX driveMotor,
            TalonFX turnMotor,
            SimpleMotorFeedforward ff,
            CANcoder absEncoder,
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
        this.nominalVoltage = nominalVoltage;
        this.lastAngle = getState().angle.getDegrees();
        resetToAbsolute();
    }

    public double getDrivePos() {
        return driveMotor.getRotorPosition().getValue() * drivePositionConversion;
    }

    public double getTurnAngle() {
        return turnMotor.getRotorPosition().getValue() * turnPositionConversion;
    }

    public double getDriveVelocity() {
        return driveMotor.getRotorVelocity().getValue() * driveVelocityConversion;
    }

    public double getTurnVelocity() {
        return turnMotor.getRotorVelocity().getValue() * turnVelocityConversion;
    }

    public Rotation2d getAbsEncoderPos() {
        return Rotation2d.fromDegrees(absEncoder.getAbsolutePosition().getValue() * 360);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), Rotation2d.fromDegrees(getTurnAngle()));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePos(), Rotation2d.fromDegrees(getTurnAngle()));
    }

    public double getVoltage() {
        return driveMotor.getSupplyVoltage().getValue();
    }

    public void resetToAbsolute() {
        double absolutePosition = (getCanCoder() - absOffsetDeg); // / turnPositionConversion;
        SmartDashboard.putNumber("bruh", absolutePosition / 360);
        var error = turnMotor.setRotorPosition(absolutePosition / 360);

        // double absoluteAngle = (getAbsEncoderPos().getDegrees() - absOffsetDeg);
        // double adjustedAngle = CTREModuleState.optimizeAngle(absoluteAngle, getTurnAngle());
        // turnMotor.setSelectedSensorPosition(adjustedAngle / turnPositionConversion);
    }

    public double getCanCoder() {
        return absEncoder.getAbsolutePosition().getValue() * 360;
    }

    public CANcoder getCanCoderObject() {
        return this.absEncoder;
    }

    public void resetDriveEncoders() {
        driveMotor.setRotorPosition(0);
    }

    public void setNeutralMode(
            NeutralModeValue turnNeutralMode, NeutralModeValue driveNeutralMode) {
        // turnMotor.setNeutralMode(turnNeutralMode);
        // driveMotor.setNeutralMode(driveNeutralMode);
        TalonFXConfigurator turnConfigurator = turnMotor.getConfigurator();
        TalonFXConfiguration turnConfiguration = new TalonFXConfiguration();
        turnConfigurator.refresh(turnConfiguration);
        turnConfiguration.MotorOutput.NeutralMode = turnNeutralMode;
        turnConfigurator.apply(turnConfiguration);
        TalonFXConfigurator driveConfigurator = driveMotor.getConfigurator();
        TalonFXConfiguration driveConfiguration = new TalonFXConfiguration();
        driveConfigurator.refresh(driveConfiguration);
        driveConfiguration.MotorOutput.NeutralMode = driveNeutralMode;
        driveConfigurator.apply(driveConfiguration);
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
            dutyCycleOut.Output = percentOutput;
            driveMotor.setControl(dutyCycleOut);
            this.percentOut = percentOutput;
        } else {
            // MPS to falcon
            double velocity = desiredState.speedMetersPerSecond / driveVelocityConversion;
            velocityDutyCycle.Velocity = velocity;
            velocityDutyCycle.FeedForward =
                    ff.calculate(desiredState.speedMetersPerSecond) / nominalVoltage;
            driveMotor.setControl(velocityDutyCycle);
        }

        double angle =
                (Math.abs(desiredState.speedMetersPerSecond)
                                <= (SwerveDriveConstants.kDrivePossibleMaxSpeedMPS * 0.01))
                        ? lastAngle
                        : desiredState.angle.getDegrees();
        // Prevents Jittering.
        positionDutyCycle.Position = angle / turnPositionConversion;
        turnMotor.setControl(positionDutyCycle);
        lastAngle = angle;
    }

    public void stop() {
        // stop velocity, not set position to 0
        driveMotor.setControl(new DutyCycleOut(0));
        turnMotor.setControl(new DutyCycleOut(0));
    }
}
