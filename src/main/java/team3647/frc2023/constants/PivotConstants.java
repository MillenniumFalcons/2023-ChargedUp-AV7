package team3647.frc2023.constants;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.InterpolatingTreeMap;

public class PivotConstants {
    // positive is swinging towards the front of the robot
    public static final TalonFX kMaster = new TalonFX(GlobalConstants.PivotIds.kMasterId);
    public static final TalonFX kSlave = new TalonFX(GlobalConstants.PivotIds.kSlaveId);

    public static final boolean opposeMaster = false;
    // public static final Follower kSlave = new Follower(GlobalConstants.PivotIds.kMasterId,
    // false);
    public static final boolean kMasterInvert = true;
    public static final boolean kSlaveInvert = kMasterInvert;

    private static final TalonFXConfiguration kMasterConfig = new TalonFXConfiguration();

    // private static final double kGearBoxRatio = 1 / 91.022;
    private static final double kGearBoxRatio = 1 / 120.0;

    public static final double kNativePosToDegrees =
            kGearBoxRatio / GlobalConstants.kFalconTicksPerRotation * 360.0;

    public static final double kNativeVelToDPS = kNativePosToDegrees;

    public static final double kMaxVelocityTicks = (400.0 / kNativeVelToDPS) * 1.8;
    public static final double kMaxAccelerationTicks = (200.0 / kNativeVelToDPS) * 1.8;

    public static final double kMinDegree = -30.0;
    public static final double kMaxDegree = 210.0;

    // kG at max extension
    public static final double kG = 0.63;

    private static final double masterKP = 0.3;
    private static final double masterKI = 0;
    private static final double masterKD = 0;

    public static final double nominalVoltage = 11.0;
    public static final double kStallCurrent = 30.0;
    public static final double kMaxCurrent = 60.0;

    public static final double kMaxkG = 0.6;
    public static final double[][] kVoltageGravity = {{0.1, 0.1}, {Units.inchesToMeters(61), 0.55}};

    public static final InterpolatingTreeMap<Double, Double> kLengthGravityVoltageMap =
            new InterpolatingTreeMap<>();
    public static final double kInitialAngle = 90.0;

    static {
        Slot0Configs kMasterSlot0 = new Slot0Configs();
        // VoltageConfigs kMasterVoltage = new VoltageConfigs();
        CurrentLimitsConfigs kMasterCurrent = new CurrentLimitsConfigs();
        MotionMagicConfigs kMasterMotionMagic = new MotionMagicConfigs();
        MotorOutputConfigs kMasterMotorOutput = new MotorOutputConfigs();
        SoftwareLimitSwitchConfigs kMasterSoftLimit = new SoftwareLimitSwitchConfigs();
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
        kMasterMotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        kMasterSoftLimit.ForwardSoftLimitEnable = true;
        kMasterSoftLimit.ForwardSoftLimitThreshold = kMaxDegree / kNativePosToDegrees;
        kMasterSoftLimit.ReverseSoftLimitEnable = true;
        kMasterSoftLimit.ReverseSoftLimitThreshold = kMinDegree / kNativePosToDegrees;

        kMasterConfigurator.apply(kMasterSlot0);
        // kMasterConfigurator.apply(kMasterVoltage);
        kMasterConfigurator.apply(kMasterMotionMagic);
        kMasterConfigurator.apply(kMasterMotorOutput);
        kMasterConfigurator.apply(kMasterSoftLimit);

        for (double[] pair : kVoltageGravity) {
            kLengthGravityVoltageMap.put(pair[0], pair[1]);
        }
    }

    public static double getkGFromLength(double length) {
        double d = kLengthGravityVoltageMap.get(length);

        return MathUtil.clamp(d, 0.0, kMaxkG);
    }

    private PivotConstants() {}
}
