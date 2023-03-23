package team3647.frc2023.constants;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import edu.wpi.first.wpilibj.DigitalInput;

public class RollersConstants {
    public static final TalonFX kMaster = new TalonFX(GlobalConstants.RollersIds.kMasterId);
    public static final DigitalInput kCubeSensor =
            new DigitalInput(GlobalConstants.RollersIds.kSensorId);

    public static final TalonFXInvertType kMasterInvert = TalonFXInvertType.Clockwise;

    private static final TalonFXConfiguration kMasterConfig = new TalonFXConfiguration();

    public static final double kNominalVoltage = 11.0;
    public static final double kStallCurrent = 25.0;
    public static final double kMaxCurrent = 20.0;
    public static final double kCurrentLimitTimeMs = 1000;

    static {
        kMaster.configAllSettings(kMasterConfig, GlobalConstants.kTimeoutMS);

        kMaster.setNeutralMode(NeutralMode.Brake);
        kMaster.enableVoltageCompensation(true);
        kMaster.configVoltageCompSaturation(kNominalVoltage, GlobalConstants.kTimeoutMS);
        kMaster.configGetStatorCurrentLimit(
                new StatorCurrentLimitConfiguration(true, kStallCurrent, kMaxCurrent, 5));
        kMaster.setInverted(kMasterInvert);
    }

    private RollersConstants() {}
}
