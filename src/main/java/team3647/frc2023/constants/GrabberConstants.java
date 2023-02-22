package team3647.frc2023.constants;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;

public class GrabberConstants {
    public static final Solenoid pistons =
            new Solenoid(GlobalConstants.kPCMType, GlobalConstants.GrabberIds.pistonChannel);
    public static final DigitalInput gamePieceSensor =
            new DigitalInput(GlobalConstants.GrabberIds.sensorPort);
    public static final TalonFX kMaster = new TalonFX(GlobalConstants.GrabberIds.masterId);
    public static final TalonFXConfiguration kMasterConfig = new TalonFXConfiguration();
    // need conversion to real values (deg and m/s here)
    public static final double kP = 0.5;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final double gearBoxReduction = 1.0;
    public static final double kNativePosToMeters = 1;
    // kOutputRotationMeters / GlobalConstants.kFalconTicksPerRotation;
    public static final double kNativeVelToMpS = 10 * kNativePosToMeters;
    public static final double kNominalVoltage = 11;

    static {
        kMaster.configFactoryDefault();
        kMasterConfig.voltageCompSaturation = kNominalVoltage;
        kMasterConfig.slot0.kP = kP;
        kMasterConfig.slot0.kI = kI;
        kMasterConfig.slot0.kD = kD;
        kMaster.configAllSettings(kMasterConfig);
        kMaster.enableVoltageCompensation(true);
        kMaster.setNeutralMode(NeutralMode.Brake);
    }
}
