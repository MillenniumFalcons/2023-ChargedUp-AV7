package team3647.frc2023.constants;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.InterpolatingTreeMap;

public class PivotConstants {
    // positive is swinging towards the front of the robot
    public static final TalonFX kMaster = new TalonFX(GlobalConstants.PivotIds.kMasterId);
    public static final TalonFX kSlave = new TalonFX(GlobalConstants.PivotIds.kSlaveId);

    private static final TalonFXConfiguration kMasterConfig = new TalonFXConfiguration();

    private static final double kGearBoxRatio = 1 / 126.0;

    public static final double kNativePosToDegrees =
            kGearBoxRatio / GlobalConstants.kFalconTicksPerRotation * 360.0;

    public static final double kNativeVelToDPS = 10 * kNativePosToDegrees;

    public static final double kMaxVelocityTicks = 45.0 / kNativeVelToDPS;
    public static final double kMaxAccelerationTicks = 45.0 / kNativeVelToDPS;

    public static final double kG = 0.55;

    private static final double masterKP = 0.1;
    private static final double masterKI = 0;
    private static final double masterKD = 0;

    public static final double nominalVoltage = 11.0;
    public static final double kStallCurrent = 30.0;
    public static final double kMaxCurrent = 60.0;

    public static final double kMaxkG = 0.6;
    public static final double[][] kVoltageGravity = {{0.1, 0.1}, {Units.inchesToMeters(61), 0.55}};

    public static final InterpolatingTreeMap<Double, Double> kLengthGravityVoltageMap =
            new InterpolatingTreeMap<Double, Double>();

    static {
        kMaster.configFactoryDefault();
        kSlave.configFactoryDefault();

        kMasterConfig.slot0.kP = masterKP;
        kMasterConfig.slot0.kI = masterKI;
        kMasterConfig.slot0.kD = masterKD;
        kMasterConfig.voltageCompSaturation = nominalVoltage;
        kMasterConfig.motionAcceleration = kMaxVelocityTicks;
        kMasterConfig.motionCruiseVelocity = kMaxAccelerationTicks;

        kMaster.configAllSettings(kMasterConfig, GlobalConstants.kTimeoutMS);
        kSlave.follow(kMaster);
        kMaster.configGetStatorCurrentLimit(
                new StatorCurrentLimitConfiguration(true, kStallCurrent, kMaxCurrent, 3));

        kMaster.setNeutralMode(NeutralMode.Brake);
        kSlave.setNeutralMode(NeutralMode.Brake);
        kMaster.enableVoltageCompensation(true);
        kSlave.enableVoltageCompensation(true);
    }

    public static double getkGFromLength(double length) {
        double d = kLengthGravityVoltageMap.get(length);

        return MathUtil.clamp(d, 0.0, kMaxkG);
    }
}
