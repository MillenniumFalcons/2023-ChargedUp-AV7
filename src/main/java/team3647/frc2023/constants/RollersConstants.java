package team3647.frc2023.constants;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

public class RollersConstants {
    public static final TalonFX kMaster = new TalonFX(GlobalConstants.RollersIds.kMasterId);

    public static final InvertType kMasterInvert = InvertType.InvertMotorOutput;

    private static final TalonFXConfiguration kMasterConfig = new TalonFXConfiguration();

    public static final double kNominalVoltage = 11.0;
    public static final double kStallCurrent = 20.0;
    public static final double kMaxCurrent = 60.0;
    public static final double kCurrentLimitTimeMs = 1000;

    static {
        // kMasterConfig.statorCurrLimit =
        //         new StatorCurrentLimitConfiguration(
        //                 true, kStallCurrent, kMaxCurrent, kCurrentLimitTimeMs);

        kMaster.configAllSettings(kMasterConfig, GlobalConstants.kTimeoutMS);

        kMaster.setNeutralMode(NeutralMode.Coast);
        kMaster.enableVoltageCompensation(true);
        kMaster.configVoltageCompSaturation(kNominalVoltage, GlobalConstants.kTimeoutMS);
    }

    private RollersConstants() {}
}
