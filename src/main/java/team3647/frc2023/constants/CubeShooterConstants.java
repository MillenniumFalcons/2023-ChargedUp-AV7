package team3647.frc2023.constants;

// import com.ctre.phoenix.motorcontrol.NeutralMode;
// import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
// import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
// import com.ctre.phoenix.motorcontrol.can.TalonFX;
// import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;

public class CubeShooterConstants {
    public static final TalonFX kTopRoller =
            new TalonFX(GlobalConstants.CubeShooterIds.kTopMasterId);
    public static final TalonFX kBottomRoller =
            new TalonFX(GlobalConstants.CubeShooterIds.kBottomMasterId);
    private static final TalonFXConfiguration kMasterConfig = new TalonFXConfiguration();

    public static final boolean kTopMotorInvert = true;
    public static final boolean kBottomMotorInvert = true;

    // kG at max extension
    public static final double kNominalVoltage = 11.0;
    public static final double kStallCurrent = 25.0;
    public static final double kMaxCurrent = 20.0;

    static {
        MotorOutputConfigs kTopMotor = new MotorOutputConfigs();
        MotorOutputConfigs kBottomMotor = new MotorOutputConfigs();

        // VoltageConfigs kTopVoltage = new VoltageConfigs();
        // VoltageConfigs kBottomVoltage = new VoltageConfigs();

        CurrentLimitsConfigs kTopCurrent = new CurrentLimitsConfigs();
        CurrentLimitsConfigs kBottomCurrent = new CurrentLimitsConfigs();

        TalonFXConfigurator kTopRollerConfigurator = kTopRoller.getConfigurator();
        TalonFXConfigurator kBottomRollerConfigurator = kBottomRoller.getConfigurator();

        kTopRollerConfigurator.apply(kMasterConfig);

        kTopMotor.NeutralMode = NeutralModeValue.Brake;
        kBottomMotor.NeutralMode = NeutralModeValue.Brake;
        kTopMotor.Inverted = InvertedValue.Clockwise_Positive;
        kBottomMotor.Inverted = InvertedValue.Clockwise_Positive;

        kTopCurrent.StatorCurrentLimitEnable = false;
        kBottomCurrent.StatorCurrentLimitEnable = false;
        // kTopCurrent.StatorCurrentLimit = kMaxCurrent;
        // kBottomCurrent.StatorCurrentLimit = kMaxCurrent;

        kTopRollerConfigurator.apply(kTopMotor);
        kTopRollerConfigurator.apply(kTopCurrent);

        kBottomRollerConfigurator.apply(kTopMotor);
        // kBottomRollerConfigurator.apply(kTopVoltage);
        kBottomRollerConfigurator.apply(kBottomCurrent);
    }
}
