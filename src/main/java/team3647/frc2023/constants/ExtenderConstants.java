package team3647.frc2023.constants;

// import com.ctre.phoenix.motorcontrol.InvertType;
// import com.ctre.phoenix.motorcontrol.NeutralMode;
// import com.ctre.phoenix.motorcontrol.can.TalonFX;
// import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;

public class ExtenderConstants {
    public static final TalonFX kMaster = new TalonFX(GlobalConstants.ExtenderIds.kMasterId);

    public static final TalonFXConfiguration kMasterConfig = new TalonFXConfiguration();
    public static final boolean kMasterInvert = true;

    public static final double kRevTicksSoftLimit =
            2000.0
                    / GlobalConstants.kFalcon5TicksPerRotation
                    / GlobalConstants.kFalconTicksPerRotation;

    private static final double kGearBoxRatio = 14.0 / 48.0 * 30.0 / 40.0 * 18.0 / 24.0;
    private static final double kDrumDiameterMeters = Units.inchesToMeters(1.2);

    public static final double kOutputRotationMeters =
            kDrumDiameterMeters * Math.PI * kGearBoxRatio;
    public static final double kNativePosToMeters = 1;
    // kOutputRotationMeters / GlobalConstants.kFalconTicksPerRotation;
    public static final double kNativeVelToMpS = kNativePosToMeters;

    public static final double kMaxVelocityTicks =
            30000.0
                    / GlobalConstants.kFalcon5TicksPerRotation
                    * 3.0
                    / GlobalConstants.kFalconTicksPerRotation;
    public static final double kMaxAccelerationTicks =
            30000.0
                    / GlobalConstants.kFalcon5TicksPerRotation
                    * 3.0
                    / GlobalConstants.kFalconTicksPerRotation;

    public static final double kMinimumPositionTicks =
            2000
                    / GlobalConstants.kFalcon5TicksPerRotation
                    / GlobalConstants.kFalconTicksPerRotation;
    public static final double kMaximumPositionTicks =
            56000.0
                    / GlobalConstants.kFalcon5TicksPerRotation
                    / GlobalConstants.kFalconTicksPerRotation;

    public static final double kS = 0.0;
    public static final double kV = 0.0;
    public static final double kA = 0.0;

    public static final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV, kA);

    private static final double kP = 0.1;
    private static final double kI = 0.0;
    private static final double kD = 0.0;

    public static final double nominalVoltage = 11.0;

    /** ticks */
    public static final double kLevelTwoExtendCone = 22601.0; // 32000 * 0.75;
    /** ticks */
    public static final double kLevelThreeExtendCone = 53760.0; // 78500 * 0.75;
    /** ticks */
    public static final double kLevelTwoExtendCube = 0; // 30000 * 0.75;
    /** ticks */
    public static final double kLevelThreeExtendCube = 31929.0; // 74000 * 0.75;

    public static final double kDoubleStation = 8600.0;

    static {
        Slot0Configs kMasterSlot0 = new Slot0Configs();
        // VoltageConfigs kMasterVoltage = new VoltageConfigs();
        MotionMagicConfigs kMasterMotionMagic = new MotionMagicConfigs();
        MotorOutputConfigs kMasterMotorOutput = new MotorOutputConfigs();
        SoftwareLimitSwitchConfigs kMasterSoftLimit = new SoftwareLimitSwitchConfigs();
        TalonFXConfigurator kMasterConfigurator = kMaster.getConfigurator();
        kMasterConfigurator.apply(kMasterConfig);

        kMasterSlot0.kP = kP;
        kMasterSlot0.kI = kI;
        kMasterSlot0.kD = kD;
        // kMasterVoltage.PeakForwardVoltage = nominalVoltage;
        // kMasterVoltage.PeakReverseVoltage = nominalVoltage;
        kMasterMotionMagic.MotionMagicAcceleration = kMaxVelocityTicks;
        kMasterMotionMagic.MotionMagicCruiseVelocity = kMaxAccelerationTicks;
        kMasterMotorOutput.PeakReverseDutyCycle = -0.7;
        kMasterMotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        kMasterMotorOutput.NeutralMode = NeutralModeValue.Brake;
        kMasterSoftLimit.ReverseSoftLimitEnable = true;
        kMasterSoftLimit.ReverseSoftLimitThreshold = kRevTicksSoftLimit;

        kMasterConfigurator.apply(kMasterSlot0);
        // kMasterConfigurator.apply(kMasterVoltage);
        kMasterConfigurator.apply(kMasterMotionMagic);
        kMasterConfigurator.apply(kMasterMotorOutput);
        kMasterConfigurator.apply(kMasterSoftLimit);
    }
}
