package team3647.frc2023.constants;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

public class GrabberConstants {
    public static final TalonFX kMaster = new TalonFX(GlobalConstants.ExtenderIds.kMasterId);

    public static final TalonFXConfiguration kMasterConfig = new TalonFXConfiguration();

    private static final double kGearBoxRatio = 14.0 / 64.0 * 24.0 / 64.0;

    public static final double kNativePosToDegrees =
            kGearBoxRatio / GlobalConstants.kFalconTicksPerRotation * 360.0;

    public static final double kNativeVelToDPS = 10 * kNativePosToDegrees;

    private static final double kP = 0.0;
    private static final double kI = 0.0;
    private static final double kD = 0.0;

    public static final double nominalVoltage = 11.0;

    static {
        kMaster.configFactoryDefault();

        kMasterConfig.slot0.kP = kP;
        kMasterConfig.slot0.kI = kI;
        kMasterConfig.slot0.kD = kD;

        kMaster.setNeutralMode(NeutralMode.Coast);
    }
}
