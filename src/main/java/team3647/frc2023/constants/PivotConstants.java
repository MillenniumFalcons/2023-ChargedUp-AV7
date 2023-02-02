package team3647.frc2023.constants;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

public class PivotConstants {
    // positive is swinging towards the front of the robot
    public static final TalonFX kMaster = new TalonFX(GlobalConstants.PivotIds.kMasterId);
    public static final TalonFX kSlave = new TalonFX(GlobalConstants.PivotIds.kSlaveId);

    private static final TalonFXConfiguration kMasterConfig = new TalonFXConfiguration();

    private static final double kGearBoxRatio = 1 / 126.0;

    public static final double kNativePosToDegrees =
            kGearBoxRatio / GlobalConstants.kFalconTicksPerRotation * 360.0;

    public static final double kNativeVelToDPS = 10 * kNativePosToDegrees;

    private static final double masterKP = 0;
    private static final double masterKI = 0;
    private static final double masterKD = 0;

    public static final double nominalVoltage = 11.0;

    static {
        kMaster.configFactoryDefault();
        kSlave.configFactoryDefault();

        kMasterConfig.slot0.kP = masterKP;
        kMasterConfig.slot0.kI = masterKI;
        kMasterConfig.slot0.kD = masterKD;

        kSlave.follow(kMaster);
        kMaster.configAllSettings(kMasterConfig);
        kSlave.configAllSettings(kMasterConfig);

        kMaster.setNeutralMode(NeutralMode.Brake);
        kSlave.setNeutralMode(NeutralMode.Brake);
    }
}
