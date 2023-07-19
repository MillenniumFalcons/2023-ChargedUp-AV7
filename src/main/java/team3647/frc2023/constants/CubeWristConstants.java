package team3647.frc2023.constants;

// import com.ctre.phoenix.motorcontrol.InvertType;
// import com.ctre.phoenix.motorcontrol.NeutralMode;
// import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
// import com.ctre.phoenix.motorcontrol.can.TalonFX;
// import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import com.playingwithfusion.TimeOfFlight;

public class CubeWristConstants {
    public static final TalonFX kMaster = new TalonFX(GlobalConstants.CubeWristIds.kMasterId);
    public static final TalonFXConfiguration kMasterConfig = new TalonFXConfiguration();
    public static final TimeOfFlight timeOfFlight =
            new TimeOfFlight(GlobalConstants.CubeWristIds.timeOfFlightId);

    public static final boolean kMasterInvert = false;

    private static final double kGearBoxRatio = 12 / 76.0 * 18 / 80.0 * 18 / 48.0;

    public static final double kNativePosToDegrees =
            kGearBoxRatio / GlobalConstants.kFalconTicksPerRotation * 360.0;

    public static final double kNativeVelToDPS = kNativePosToDegrees;

    public static final double kMaxVelocityTicks = (2000.0 / kNativeVelToDPS);
    public static final double kMaxAccelerationTicks = (4000.0 / kNativeVelToDPS);

    public static final double kInitialDegree = 0.0;
    public static final double kMinDegree = 0.0;
    public static final double kMaxDegree = 100.0;

    public static final double masterKP = 1.0;
    public static final double masterKI = 0.0;
    public static final double masterKD = 0.0;

    public static final double nominalVoltage = 11.0;
    public static final double kStallCurrent = 30.0;
    public static final double kMaxCurrent = 60.0;

    // kG at max extension
    public static final double kG = 0.00;

    private static final Slot0Configs kMasterSlot0 = new Slot0Configs();
    // VoltageConfigs kMasterVoltage = new VoltageConfigs();
    private static final CurrentLimitsConfigs kMasterCurrent = new CurrentLimitsConfigs();
    private static final MotionMagicConfigs kMasterMotionMagic = new MotionMagicConfigs();
    private static final MotorOutputConfigs kMasterMotorOutput = new MotorOutputConfigs();
    private static final SoftwareLimitSwitchConfigs kMasterSoftLimit =
            new SoftwareLimitSwitchConfigs();

    static {
        TalonFXConfigurator kMasterConfigurator = kMaster.getConfigurator();
        kMasterConfigurator.apply(kMasterConfig);

        kMasterSlot0.kP = masterKP;
        kMasterSlot0.kI = masterKI;
        kMasterSlot0.kD = masterKD;
        // kMasterVoltage.PeakForwardVoltage = nominalVoltage;
        // kMasterVoltage.PeakReverseVoltage = nominalVoltage;
        kMasterCurrent.StatorCurrentLimitEnable = true;
        kMasterCurrent.StatorCurrentLimit = kMaxCurrent;
        kMasterMotionMagic.MotionMagicAcceleration = kMaxVelocityTicks;
        kMasterMotionMagic.MotionMagicCruiseVelocity = kMaxAccelerationTicks;
        kMasterMotorOutput.PeakReverseDutyCycle = -0.5;
        kMasterMotorOutput.NeutralMode = NeutralModeValue.Coast;
        kMasterMotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        kMasterSoftLimit.ForwardSoftLimitEnable = true;
        kMasterSoftLimit.ForwardSoftLimitThreshold = kMaxDegree / kNativePosToDegrees;
        kMasterSoftLimit.ReverseSoftLimitEnable = true;
        kMasterSoftLimit.ReverseSoftLimitThreshold = kMinDegree / kNativePosToDegrees;

        kMasterConfigurator.apply(kMasterSlot0);
        // kMasterConfigurator.apply(kMasterVoltage);
        kMasterConfigurator.apply(kMasterMotionMagic);
        kMasterConfigurator.apply(kMasterMotorOutput);
        kMasterConfigurator.apply(kMasterSoftLimit);
    }
}
