package team3647.frc2023.constants;

import com.ctre.phoenix6.StatusCode;
// import com.ctre.phoenix.ErrorCode;
// import com.ctre.phoenix.motorcontrol.NeutralMode;
// import com.ctre.phoenix.motorcontrol.can.TalonFX;
// import com.ctre.phoenix.motorcontrol.can.TalonFXConfigurator;
// import com.ctre.phoenix.sensors.AbsoluteSensorRange;
// import com.ctre.phoenix.sensors.CANCoder;
// import com.ctre.phoenix.sensors.CANCoderConfiguration;
// import com.ctre.phoenix.sensors.Pigeon2;
// import com.ctre.phoenix.sensors.Pigeon2Configuration;
// import com.ctre.phoenix.sensors.SensorInitializationStrategy;
// import com.ctre.phoenix.sensors.SensorTimeBase;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import team3647.lib.SwerveModule;

public class SwerveDriveConstants {
    // default falcon rotates counter clockwise (CCW)
    // make sure gyro -CW, +CCW
    public static final SensorDirectionValue canCoderInvert =
            SensorDirectionValue.CounterClockwise_Positive;
    public static final InvertedValue kDriveMotorInverted = InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue kTurnMotorInverted = InvertedValue.Clockwise_Positive;

    // physical possible max speed
    public static final double kDrivePossibleMaxSpeedMPS = 5;
    public static final double kRotPossibleMaxSpeedRadPerSec = 10;

    public static final NeutralModeValue kTurnNeutralMode = NeutralModeValue.Coast;
    public static final NeutralModeValue kDriveNeutralMode = NeutralModeValue.Brake;

    public static final TalonFX kFrontLeftDrive =
            new TalonFX(GlobalConstants.SwerveDriveIds.kFrontLeftDriveId, "rio");
    public static final TalonFX kFrontLeftTurn =
            new TalonFX(GlobalConstants.SwerveDriveIds.kFrontLeftTurnId, "rio");
    public static final CANcoder kFrontLeftAbsEncoder =
            new CANcoder(GlobalConstants.SwerveDriveIds.kFrontLeftAbsEncoderPort, "rio");

    public static final TalonFX kFrontRightDrive =
            new TalonFX(GlobalConstants.SwerveDriveIds.kFrontRightDriveId, "rio");
    public static final TalonFX kFrontRightTurn =
            new TalonFX(GlobalConstants.SwerveDriveIds.kFrontRightTurnId, "rio");
    public static final CANcoder kFrontRightAbsEncoder =
            new CANcoder(GlobalConstants.SwerveDriveIds.kFrontRightAbsEncoderPort, "rio");

    public static final TalonFX kBackLeftDrive =
            new TalonFX(GlobalConstants.SwerveDriveIds.kBackLeftDriveId, "rio");
    public static final TalonFX kBackLeftTurn =
            new TalonFX(GlobalConstants.SwerveDriveIds.kBackLeftTurnId, "rio");
    public static final CANcoder kBackLeftAbsEncoder =
            new CANcoder(GlobalConstants.SwerveDriveIds.kBackLeftAbsEncoderPort, "rio");

    public static final TalonFX kBackRightDrive =
            new TalonFX(GlobalConstants.SwerveDriveIds.kBackRightDriveId, "rio");
    public static final TalonFX kBackRightTurn =
            new TalonFX(GlobalConstants.SwerveDriveIds.kBackRightTurnId, "rio");
    public static final CANcoder kBackRightAbsEncoder =
            new CANcoder(GlobalConstants.SwerveDriveIds.kBackRightAbsEncoderPort, "rio");

    public static final Pigeon2 kGyro = new Pigeon2(GlobalConstants.SwerveDriveIds.gyroPin, "rio");

    // config swerve module reversed here, module class doens't reverse for you

    // distance between right and left wheels
    public static final double kTrackWidth = Units.inchesToMeters(20.75);
    // distance between front and back wheels

    public static final double kWheelBase = Units.inchesToMeters(20.75);
    // translations are locations of each module wheel
    // 0 --> ++ --> front left
    // 1 --> +- --> front right
    // 2 --> -+ --> back left
    // 3 --> -- --> back right
    // c is center of robot,
    // +x towards front of robot, +y towards left of robot
    //         +x
    //         ^
    //         |
    //    +y<--c
    public static final SwerveDriveKinematics kDriveKinematics =
            new SwerveDriveKinematics(
                    new Translation2d(kWheelBase / 2.0, kTrackWidth / 2.0),
                    new Translation2d(kWheelBase / 2.0, -kTrackWidth / 2.0),
                    new Translation2d(-kWheelBase / 2.0, kTrackWidth / 2.0),
                    new Translation2d(-kWheelBase / 2.0, -kTrackWidth / 2.0));

    //  config conversion factors here for each module. in meters for postiion and radians for
    // rotation.

    // from motor to output shaft
    public static final double kDriveMotorGearRatio = 1 / 6.75;
    public static final double kTurnMotorGearRatio = 7.0 / 150.0;
    public static final double kWheelDiameterMeters = 0.097; // 97mm

    //     // divide for tick to deg
    public static final double kTurnMotorNativeToDeg = kTurnMotorGearRatio * 360.0;

    public static final double kTurnMotorNativeToDPS = kTurnMotorNativeToDeg; // RPS / Native/10ms

    public static final double kWheelRotationToMetersDrive =
            kWheelDiameterMeters * Math.PI * kDriveMotorGearRatio;

    // Multiply by 10 because velocity is in ticks/100ms
    public static final double kFalconVelocityToMpS = kWheelRotationToMetersDrive;

    public static final double kFalconTicksToMeters = kWheelRotationToMetersDrive;

    public static final double kNominalVoltage = 10;
    public static final double kStallCurrent = 35;
    public static final double kMaxCurrent = 60;

    // prac bot
    //     public static final double kAbsFrontLeftEncoderOffsetDeg = 302.52;
    //     public static final double kAbsFrontRightEncoderOffsetDeg = 244.77;
    //     public static final double kAbsBackLeftEncoderOffsetDeg = 121.9;
    //     public static final double kAbsBackRightEncoderOffsetDeg = 240.3;

    // comp bot
    //     public static final double kAbsFrontLeftEncoderOffsetDeg = 37.01;
    //     public static final double kAbsFrontRightEncoderOffsetDeg = 184.48;
    //     public static final double kAbsBackLeftEncoderOffsetDeg = 348.13;
    //     public static final double kAbsBackRightEncoderOffsetDeg = 246.88;

    // comp bot
    public static final double kAbsFrontLeftEncoderOffsetDeg = 123.13; // 215.77; //35.595;
    public static final double kAbsFrontRightEncoderOffsetDeg = 64.51; // 183.51; //182.724;
    public static final double kAbsBackLeftEncoderOffsetDeg = 291.36; // 347.34; //348.222;
    public static final double kAbsBackRightEncoderOffsetDeg = 61.26; // 67.23; //247.851;

    // max speed limits that we want
    public static final double kTeleopDriveMaxAccelUnitsPerSec = kDrivePossibleMaxSpeedMPS / 2;
    public static final double kTeleopDriveMaxAngularAccelUnitsPerSec =
            kRotPossibleMaxSpeedRadPerSec / 3;

    public static final TalonFXConfigurator kFrontLeftDriveConfig =
            kFrontLeftDrive.getConfigurator();
    public static final TalonFXConfigurator kFrontLeftTurnConfig = kFrontLeftTurn.getConfigurator();

    public static final TalonFXConfigurator kFrontRightDriveConfig =
            kFrontRightDrive.getConfigurator();
    public static final TalonFXConfigurator kFrontRightTurnConfig =
            kFrontRightTurn.getConfigurator();

    public static final TalonFXConfigurator kBackLeftDriveConfig = kBackLeftDrive.getConfigurator();
    public static final TalonFXConfigurator kBackLeftTurnConfig = kBackLeftTurn.getConfigurator();

    public static final TalonFXConfigurator kBackRightDriveConfig =
            kBackRightDrive.getConfigurator();
    public static final TalonFXConfigurator kBackRightTurnConfig = kBackRightTurn.getConfigurator();

    public static final CANcoderConfigurator kFrontLeftAbsConfig =
            kFrontLeftAbsEncoder.getConfigurator();
    public static final CANcoderConfigurator kFrontRightAbsConfig =
            kFrontRightAbsEncoder.getConfigurator();
    public static final CANcoderConfigurator kBackLeftAbsConfig =
            kBackLeftAbsEncoder.getConfigurator();
    public static final CANcoderConfigurator kBackRightAbsConfig =
            kBackRightAbsEncoder.getConfigurator();

    public static final Pigeon2Configurator kGyroConfig = kGyro.getConfigurator();

    // master FF for drive for all modules
    public static final double kS = (0.56744 / 12); // 0.56744; // Volts
    public static final double kV = (2.5 / 12.0); // Volts
    public static final double kA = (0.0 / 12); // Volts

    public static final SimpleMotorFeedforward kMasterDriveFeedforward =
            new SimpleMotorFeedforward(kS, kV, kA);

    // master PID constants for turn and drive for all modules
    public static final double kDriveP = 0.00014;
    public static final double kDriveI = 0.0;
    public static final double kDriveD = 0.0;

    public static final double kTurnP = 0.4;
    public static final double kTurnI = 0.0;
    public static final double kTurnD = 0;

    public static final double kYP = 1;
    public static final double kYI = 0.0;
    public static final double kYD = 0;

    public static final PIDController kYController = new PIDController(kYP, kYI, kYD);

    public static final PIDController kAutoSteerXYPIDController = new PIDController(0.05, 0, 0);
    // 3*Pi = move at 10 rads per second if we are 180* away from target heading
    public static final PIDController kAutoSteerHeadingController = new PIDController(0.03, 0, 0);
    // PID constants for roll and yaw

    // is stored as reference?
    public static final SwerveModule kFrontLeftModule =
            new SwerveModule(
                    kFrontLeftDrive,
                    kFrontLeftTurn,
                    kMasterDriveFeedforward,
                    kFrontLeftAbsEncoder,
                    kAbsFrontLeftEncoderOffsetDeg,
                    kFalconVelocityToMpS,
                    kFalconTicksToMeters,
                    kTurnMotorNativeToDPS,
                    kTurnMotorNativeToDeg,
                    kNominalVoltage);
    public static final SwerveModule kFrontRightModule =
            new SwerveModule(
                    kFrontRightDrive,
                    kFrontRightTurn,
                    kMasterDriveFeedforward,
                    kFrontRightAbsEncoder,
                    kAbsFrontRightEncoderOffsetDeg,
                    kFalconVelocityToMpS,
                    kFalconTicksToMeters,
                    kTurnMotorNativeToDPS,
                    kTurnMotorNativeToDeg,
                    kNominalVoltage);
    public static final SwerveModule kBackLeftModule =
            new SwerveModule(
                    kBackLeftDrive,
                    kBackLeftTurn,
                    kMasterDriveFeedforward,
                    kBackLeftAbsEncoder,
                    kAbsBackLeftEncoderOffsetDeg,
                    kFalconVelocityToMpS,
                    kFalconTicksToMeters,
                    kTurnMotorNativeToDPS,
                    kTurnMotorNativeToDeg,
                    kNominalVoltage);
    public static final SwerveModule kBackRightModule =
            new SwerveModule(
                    kBackRightDrive,
                    kBackRightTurn,
                    kMasterDriveFeedforward,
                    kBackRightAbsEncoder,
                    kAbsBackRightEncoderOffsetDeg,
                    kFalconVelocityToMpS,
                    kFalconTicksToMeters,
                    kTurnMotorNativeToDPS,
                    kTurnMotorNativeToDeg,
                    kNominalVoltage);

    private static void setTurnMotorConfig(
            TalonFXConfigurator configurator,
            Slot0Configs slot0,
            CurrentLimitsConfigs supplyCurrLimit,
            MotorOutputConfigs motorConfigs) {
        slot0.kP = kTurnP;
        slot0.kI = kTurnI;
        slot0.kD = kTurnD;
        supplyCurrLimit.SupplyCurrentLimitEnable = true;
        supplyCurrLimit.SupplyCurrentThreshold = kMaxCurrent;
        supplyCurrLimit.SupplyTimeThreshold = 5;
        motorConfigs.NeutralMode = kTurnNeutralMode;
        motorConfigs.Inverted = kTurnMotorInverted;
        configurator.apply(slot0);
        configurator.apply(supplyCurrLimit);
        configurator.apply(motorConfigs);
        // config.voltageCompSaturation = kNominalVoltage;
        // config.initializationStrategy = SensorInitializationStrategy.BootToZero;
    }

    private static void setDriveMotorConfig(
            TalonFXConfigurator configurator,
            Slot0Configs slot0,
            CurrentLimitsConfigs supplyCurrLimit,
            MotorOutputConfigs motorConfigs) {
        slot0.kP = kDriveP;
        slot0.kI = kDriveI;
        slot0.kD = kDriveD;
        supplyCurrLimit.SupplyCurrentLimitEnable = false;
        supplyCurrLimit.SupplyCurrentThreshold = kMaxCurrent;
        supplyCurrLimit.SupplyTimeThreshold = 5;
        motorConfigs.NeutralMode = kDriveNeutralMode;
        motorConfigs.Inverted = kDriveMotorInverted;
        configurator.apply(slot0);
        configurator.apply(supplyCurrLimit);
        configurator.apply(motorConfigs);
        // config.initializationStrategy = SensorInitializationStrategy.BootToZero;
        // config.voltageCompSaturation = kNominalVoltage;
    }

    private static void setAbsConfig(
            CANcoderConfigurator configurator, MagnetSensorConfigs magnetConfigs) {
        magnetConfigs.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        magnetConfigs.SensorDirection = canCoderInvert;
        configurator.apply(magnetConfigs);
    }

    private static void setGyroConfig(
            Pigeon2Configurator configurator, MountPoseConfigs mountPoseConfigs) {
        mountPoseConfigs.MountPoseYaw = 90;
        configurator.apply(mountPoseConfigs);
    }

    private static void printError(StatusCode error) {
        if (error.value == 0) {
            return;
        }

        System.out.println(error);
    }

    static {
        // kGyroConfig.ZAxisGyroError = 0.3;
        // // remove later
        // kGyroConfig.MountPoseYaw = 90;
        // printError(kGyro.configAllSettings(kGyroConfig, GlobalConstants.kTimeoutMS));

        setGyroConfig(kGyroConfig, new com.ctre.phoenix6.configs.MountPoseConfigs());

        setTurnMotorConfig(
                kFrontLeftTurnConfig,
                new Slot0Configs(),
                new CurrentLimitsConfigs(),
                new MotorOutputConfigs());
        setTurnMotorConfig(
                kFrontRightTurnConfig,
                new Slot0Configs(),
                new CurrentLimitsConfigs(),
                new MotorOutputConfigs());
        setTurnMotorConfig(
                kBackLeftTurnConfig,
                new Slot0Configs(),
                new CurrentLimitsConfigs(),
                new MotorOutputConfigs());
        setTurnMotorConfig(
                kBackRightTurnConfig,
                new Slot0Configs(),
                new CurrentLimitsConfigs(),
                new MotorOutputConfigs());

        setDriveMotorConfig(
                kFrontLeftDriveConfig,
                new Slot0Configs(),
                new CurrentLimitsConfigs(),
                new MotorOutputConfigs());
        setDriveMotorConfig(
                kFrontRightDriveConfig,
                new Slot0Configs(),
                new CurrentLimitsConfigs(),
                new MotorOutputConfigs());
        setDriveMotorConfig(
                kBackLeftDriveConfig,
                new Slot0Configs(),
                new CurrentLimitsConfigs(),
                new MotorOutputConfigs());
        setDriveMotorConfig(
                kBackRightDriveConfig,
                new Slot0Configs(),
                new CurrentLimitsConfigs(),
                new MotorOutputConfigs());

        setAbsConfig(kFrontLeftAbsConfig, new MagnetSensorConfigs());
        setAbsConfig(kFrontRightAbsConfig, new MagnetSensorConfigs());
        setAbsConfig(kBackLeftAbsConfig, new MagnetSensorConfigs());
        setAbsConfig(kBackRightAbsConfig, new MagnetSensorConfigs());
    }

    private SwerveDriveConstants() {}
}
