package team3647.frc2023.constants;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;

public class ExtenderConstants {
    public static final TalonFX kMaster = new TalonFX(GlobalConstants.ExtenderIds.kMasterId);
    public static final DigitalInput resetSensor =
            new DigitalInput(GlobalConstants.ExtenderIds.resetSensor);
    public static final TalonFXConfiguration kMasterConfig = new TalonFXConfiguration();
    public static final InvertType kMasterInvert = InvertType.InvertMotorOutput;

    public static final double kRevTicksSoftLimit = 2000;

    private static final double kGearBoxRatio = 14.0 / 48.0 * 30.0 / 40.0 * 18.0 / 24.0;
    private static final double kDrumDiameterMeters = Units.inchesToMeters(1.2);

    public static final double kOutputRotationMeters =
            kDrumDiameterMeters * Math.PI * kGearBoxRatio;
    public static final double kNativePosToMeters = 1;
    // kOutputRotationMeters / GlobalConstants.kFalconTicksPerRotation;
    public static final double kNativeVelToMpS = 10 * kNativePosToMeters;

    public static final double kMaxVelocityTicks = 30000.0;
    public static final double kMaxAccelerationTicks = 30000.0;

    public static final double kMinimumPositionTicks = 0;
    public static final double kMaximumPositionTicks = 80000.0;

    public static final double kS = 0.0;
    public static final double kV = 0.0;
    public static final double kA = 0.0;

    public static final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV, kA);

    private static final double kP = 0.1;
    private static final double kI = 0.0;
    private static final double kD = 0.0;

    public static final double nominalVoltage = 11.0;

    /** ticks */
    public static final double kLevelTwoExtendCone = 25006; // 32000 * 0.75;
    /** ticks */
    public static final double kLevelThreeExtendCone = 55870 * 1.1; // 78500 * 0.75;
    /** ticks */
    public static final double kLevelTwoExtendCube = 0; // 30000 * 0.75;
    /** ticks */
    public static final double kLevelThreeExtendCube = 31929; // 74000 * 0.75;

    public static final double kDoubleStation = 35000;

    static {
        kMaster.configFactoryDefault();

        kMasterConfig.slot0.kP = kP;
        kMasterConfig.slot0.kI = kI;
        kMasterConfig.slot0.kD = kD;
        kMasterConfig.motionAcceleration = kMaxVelocityTicks;
        kMasterConfig.motionCruiseVelocity = kMaxAccelerationTicks;
        kMasterConfig.voltageCompSaturation = nominalVoltage;
        kMasterConfig.slot0.allowableClosedloopError = 1000;

        kMasterConfig.peakOutputReverse = -0.7;

        kMasterConfig.reverseSoftLimitEnable = true;
        kMasterConfig.reverseSoftLimitThreshold = kRevTicksSoftLimit;

        kMaster.configAllSettings(kMasterConfig, GlobalConstants.kTimeoutMS);
        kMaster.setInverted(kMasterInvert);
        kMaster.enableVoltageCompensation(true);
        kMaster.setNeutralMode(NeutralMode.Coast);
    }
}
