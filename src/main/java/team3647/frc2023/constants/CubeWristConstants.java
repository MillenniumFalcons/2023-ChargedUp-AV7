package team3647.frc2023.constants;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

public class CubeWristConstants {
    public static final TalonFX kMaster = new TalonFX(GlobalConstants.CubeWristIds.kMasterId);
    public static final TalonFXConfiguration kMasterConfig = new TalonFXConfiguration();

    public static final InvertType kMasterInvert = InvertType.None;

    private static final double kGearBoxRatio = 12 / 76.0 * 18 / 80.0 * 18 / 48.0;

    public static final double kNativePosToDegrees =
            kGearBoxRatio / GlobalConstants.kFalconTicksPerRotation * 360.0;

    public static final double kNativeVelToDPS = 10 * kNativePosToDegrees;

    public static final double kMaxVelocityTicks = (2000 / kNativeVelToDPS);
    public static final double kMaxAccelerationTicks = (4000.0 / kNativeVelToDPS);

    public static final double kInitialDegree = 0.0;
    public static final double kMinDegree = 0.0;
    public static final double kMaxDegree = 100.0;

    public static final double masterKP = 0.5;
    public static final double masterKI = 0.0;
    public static final double masterKD = 0.0;

    public static final double nominalVoltage = 11.0;
    public static final double kStallCurrent = 30.0;
    public static final double kMaxCurrent = 60.0;

    // kG at max extension
    public static final double kG = 0.00;

    static {
        kMaster.configFactoryDefault();

        kMasterConfig.slot0.kP = masterKP;
        kMasterConfig.slot0.kI = masterKI;
        kMasterConfig.slot0.kD = masterKD;
        kMasterConfig.slot0.allowableClosedloopError = 100;
        kMasterConfig.voltageCompSaturation = nominalVoltage;
        kMasterConfig.motionAcceleration = kMaxVelocityTicks;
        kMasterConfig.motionCruiseVelocity = kMaxAccelerationTicks;
        kMasterConfig.peakOutputReverse = -0.5;
        kMasterConfig.reverseSoftLimitEnable = true;
        kMasterConfig.reverseSoftLimitThreshold = kMinDegree / kNativePosToDegrees;
        kMasterConfig.forwardSoftLimitEnable = true;
        kMasterConfig.forwardSoftLimitThreshold = kMaxDegree / kNativePosToDegrees;

        kMaster.configAllSettings(kMasterConfig, GlobalConstants.kTimeoutMS);
        kMaster.setInverted(kMasterInvert);
        kMaster.configGetStatorCurrentLimit(
                new StatorCurrentLimitConfiguration(true, kStallCurrent, kMaxCurrent, 3));

        kMaster.setNeutralMode(NeutralMode.Brake);
        kMaster.enableVoltageCompensation(true);
    }
}
