package team3647.frc2023.constants;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.wpilibj.DigitalInput;

public class RollersConstants {
    public static final TalonFX kMaster = new TalonFX(GlobalConstants.RollersIds.kMasterId);
    public static final DigitalInput kCubeSensor =
            new DigitalInput(GlobalConstants.RollersIds.kSensorId);

    // public static final TalonFXInvertType kMasterInvert = TalonFXInvertType.CounterClockwise;

    private static final TalonFXConfiguration kMasterConfig = new TalonFXConfiguration();

    public static final double kNominalVoltage = 11.0;
    public static final double kStallCurrent = 25.0;
    public static final double kMaxCurrent = 20.0;

    static {
        // VoltageConfigs kMasterVoltage = new VoltageConfigs();
        CurrentLimitsConfigs kMasterCurrent = new CurrentLimitsConfigs();
        MotorOutputConfigs kMasterMotorOutput = new MotorOutputConfigs();
        TalonFXConfigurator kMasterConfigurator = kMaster.getConfigurator();
        kMasterConfigurator.apply(kMasterConfig);

        // kMasterVoltage.PeakForwardVoltage = kNominalVoltage;
        // kMasterVoltage.PeakReverseVoltage = kNominalVoltage;
        kMasterCurrent.StatorCurrentLimitEnable = true;
        kMasterCurrent.StatorCurrentLimit = kMaxCurrent;
        kMasterMotorOutput.NeutralMode = NeutralModeValue.Brake;
        kMasterMotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        // kMasterConfigurator.apply(kMasterVoltage);
        kMasterConfigurator.apply(kMasterMotorOutput);
    }

    private RollersConstants() {}
}
