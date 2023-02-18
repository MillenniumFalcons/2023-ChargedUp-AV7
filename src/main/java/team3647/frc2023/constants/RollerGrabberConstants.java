package team3647.frc2023.constants;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import edu.wpi.first.wpilibj.Solenoid;

public class RollerGrabberConstants {
    public static final TalonFX kMaster = new TalonFX(GlobalConstants.RollerGrabberIds.masterId);
    public static final TalonFXConfiguration kMasterConfig = new TalonFXConfiguration();
    public static final double kNominalVoltage = 11;
    public static final Solenoid kPiston =
            new Solenoid(GlobalConstants.kPCMType, GlobalConstants.RollerGrabberIds.pistonChannel);

    static {
        kMaster.configFactoryDefault();
        kMasterConfig.voltageCompSaturation = kNominalVoltage;
        kMaster.configAllSettings(kMasterConfig);
        kMaster.enableVoltageCompensation(true);
    }
}
