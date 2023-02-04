package team3647.frc2023.constants;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;

public class ExtenderConstants {
    public static final TalonFX kMaster = new TalonFX(GlobalConstants.ExtenderIds.kMasterId);

    public static final TalonFXConfiguration kMasterConfig = new TalonFXConfiguration();
    public static final InvertType kMasterInvert = InvertType.InvertMotorOutput;

    public static final double kRevMetersSoftLimit = 0;

    private static final double kGearBoxRatio = 14.0 / 48.0 * 30.0 / 40.0 * 18.0 / 24.0;
    private static final double kDrumDiameterMeters = Units.inchesToMeters(1.5);

    public static final double kOutputRotationMeters =
            kDrumDiameterMeters * Math.PI * kGearBoxRatio;
    public static final double kNativePosToMeters =
            kOutputRotationMeters / GlobalConstants.kFalconTicksPerRotation;
    public static final double kNativeVelToMpS = 10 * kNativePosToMeters;

    public static final double kMaxVelocityTicks = 0.5 / kNativeVelToMpS;
    public static final double kMaxAccelerationTicks = 0.5 / kNativeVelToMpS;

    public static final double kMinimumPositionMeters = 0;
    public static final double kMaximumPositionMeters = 0.7;

    public static final double kS = 0.0;
    public static final double kV = 0.0;
    public static final double kA = 0.0;

    public static final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV, kA);

    private static final double kP = 0.1;
    private static final double kI = 0.0;
    private static final double kD = 0.0;

    public static final double nominalVoltage = 11.0;

    /** meters */
    public static final double kLevelTwoExtend = 0.5;
    /** meters */
    public static final double kLevelThreeExtend = 0.65;

    static {
        kMaster.configFactoryDefault();

        kMasterConfig.slot0.kP = kP;
        kMasterConfig.slot0.kI = kI;
        kMasterConfig.slot0.kD = kD;
        kMasterConfig.motionAcceleration = kMaxVelocityTicks;
        kMasterConfig.motionCruiseVelocity = kMaxAccelerationTicks;
        kMasterConfig.voltageCompSaturation = nominalVoltage;

        kMasterConfig.reverseSoftLimitEnable = true;
        kMasterConfig.reverseSoftLimitThreshold = kRevMetersSoftLimit / kNativePosToMeters;

        kMaster.configAllSettings(kMasterConfig, GlobalConstants.kTimeoutMS);
        kMaster.setInverted(kMasterInvert);
        kMaster.enableVoltageCompensation(true);
        kMaster.setNeutralMode(NeutralMode.Brake);
    }
}
