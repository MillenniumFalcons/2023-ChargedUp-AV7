package team3647.frc2023.constants;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;

public final class WristConstants {
    public static final TalonFX kMaster = new TalonFX(GlobalConstants.WristIds.kMasterId);
    // public static final InvertType kMasterInvert = InvertType.InvertMotorOutput;

    private static final double kGearBoxRatio = 1.0 / 70;
    private static final TalonFXConfiguration kMasterConfig = new TalonFXConfiguration();

    public static final double kNativePosToDegrees = kGearBoxRatio * 360.0;

    public static final double kNativeVelToDPS = kNativePosToDegrees;

    private static final double masterKP = 0.5;
    private static final double masterKI = 0;
    private static final double masterKD = 0;

    public static final double nominalVoltage = 11.0;
    public static final double kStallCurrent = 20.0;
    public static final double kMaxCurrent = 15;

    public static final double kG = 0.0;

    public static final double kMaxVelocityTicks = (300.0 / kNativeVelToDPS) * 8;
    public static final double kMaxAccelerationTicks = (200.0 / kNativeVelToDPS) * 8;

    // public static final double kMinDegree = 5;
    // public static final double kMaxDegree = 135;

    public static final double kMinDegree = -90;
    public static final double kMaxDegree = 135;

    public static final double kInitialDegree = 140;
    public static final double kHoldPosition = 30;
    public static final double kDoubleStationDegrees = 106;
    public static final double kConeScoreAngle = 90;
    public static final double kCubeScoreAngle = 90;

    static {
        Slot0Configs kMasterSlot0 = new Slot0Configs();
        VoltageConfigs kMasterVoltage = new VoltageConfigs();
        CurrentLimitsConfigs kMasterCurrent = new CurrentLimitsConfigs();
        MotionMagicConfigs kMasterMotionMagic = new MotionMagicConfigs();
        MotorOutputConfigs kMasterMotorOutput = new MotorOutputConfigs();
        SoftwareLimitSwitchConfigs kMasterSoftLimit = new SoftwareLimitSwitchConfigs();
        TalonFXConfigurator kMasterConfigurator = kMaster.getConfigurator();
        kMasterConfigurator.apply(kMasterConfig);

        kMasterSlot0.kP = masterKP;
        kMasterSlot0.kI = masterKI;
        kMasterSlot0.kD = masterKD;
        kMasterCurrent.StatorCurrentLimitEnable = true;
        kMasterCurrent.StatorCurrentLimit = kMaxCurrent;
        kMasterMotionMagic.MotionMagicAcceleration = kMaxVelocityTicks;
        kMasterMotionMagic.MotionMagicCruiseVelocity = kMaxAccelerationTicks;
        kMasterMotorOutput.NeutralMode = NeutralModeValue.Coast;
        kMasterMotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        kMasterSoftLimit.ForwardSoftLimitEnable = true;
        kMasterSoftLimit.ForwardSoftLimitThreshold = kMaxDegree / kNativePosToDegrees;
        kMasterSoftLimit.ReverseSoftLimitEnable = true;
        kMasterSoftLimit.ReverseSoftLimitThreshold = kMinDegree / kNativePosToDegrees;

        kMasterConfigurator.apply(kMasterSlot0);
        kMasterConfigurator.apply(kMasterVoltage);
        kMasterConfigurator.apply(kMasterMotionMagic);
        kMasterConfigurator.apply(kMasterMotorOutput);
        kMasterConfigurator.apply(kMasterSoftLimit);
    }

    private WristConstants() {}
}
