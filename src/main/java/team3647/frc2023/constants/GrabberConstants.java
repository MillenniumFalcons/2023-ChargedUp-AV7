package team3647.frc2023.constants;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class GrabberConstants {
    public static final TalonFX kMaster = new TalonFX(GlobalConstants.GrabberIds.kMasterId);

    public static final TalonFXConfiguration kMasterConfig = new TalonFXConfiguration();
    private static final double kGearBoxRatio = 14.0 / 64.0 * 24.0 / 64.0;

    public static final double kRevDegreesSoftLimit = 0;

    public static final double kNativePosToDegrees =
            kGearBoxRatio / GlobalConstants.kFalconTicksPerRotation * 360.0;

    public static final double kNativeVelToDPS = 10 * kNativePosToDegrees;

    public static final double kMaxVelocityTicks = 6000.0 / kNativeVelToDPS;
    public static final double kMaxAccelerationTicks = 6000.0 / kNativeVelToDPS;

    private static final double kS = 0.715;
    private static final double kV = 0.0;
    private static final double kA = 0.0;

    public static final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV, kA);

    private static final double kP = 0.15;
    private static final double kI = 0.0;
    private static final double kD = 0.0;

    public static final double nominalVoltage = 11.0;
    public static final double kStallCurrent = 10.0;
    public static final double kMaxCurrent = 20.0;

    static {
        kMaster.configFactoryDefault();
        kMasterConfig.slot0.kP = kP;
        kMasterConfig.slot0.kI = kI;
        kMasterConfig.slot0.kD = kD;
        kMasterConfig.voltageCompSaturation = nominalVoltage;

        kMasterConfig.reverseSoftLimitEnable = true;
        kMasterConfig.reverseSoftLimitThreshold = kRevDegreesSoftLimit / kNativePosToDegrees;

        // in native units/100ms
        kMasterConfig.motionAcceleration = kMaxVelocityTicks;
        kMasterConfig.motionCruiseVelocity = kMaxAccelerationTicks;

        kMaster.configAllSettings(kMasterConfig, GlobalConstants.kTimeoutMS);
        kMaster.configStatorCurrentLimit(
                new StatorCurrentLimitConfiguration(true, kStallCurrent, kMaxCurrent, 3));
        kMaster.setNeutralMode(NeutralMode.Brake);
        kMaster.enableVoltageCompensation(true);
    }
}
