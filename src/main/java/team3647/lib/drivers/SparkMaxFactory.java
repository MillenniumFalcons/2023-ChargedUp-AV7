package team3647.lib.drivers;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.REVLibError;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * Creates CANTalon objects and configures all the parameters we care about to factory defaults.
 * Closed-loop and sensor parameters are not set, as these are expected to be set by the
 * application. (254)
 */
public class SparkMaxFactory {
    private static final int kCANTimeout = 100;

    public static class Configuration {
        public final int CANID;
        public final boolean inverted;

        public boolean enableCurrentLimiting = false;
        public int maxFreeSpeedCurrent;
        public int maxStallCurrent;

        public IdleMode idleMode;
        public boolean voltageCompensation = false;
        public double nominalVoltage;

        public Configuration(int CANID, boolean inverted) {
            this.CANID = CANID;
            this.inverted = inverted;
        }

        public Configuration currentLimiting(
                boolean enabled, int maxFreeSpeedCurrent, int maxStallCurrent) {
            this.maxFreeSpeedCurrent = maxFreeSpeedCurrent;
            this.maxStallCurrent = maxStallCurrent;
            enableCurrentLimiting = enabled;
            return this;
        }

        public Configuration voltageCompensation(boolean enabled, double nominalVoltage) {
            this.nominalVoltage = nominalVoltage;
            voltageCompensation = enabled;
            return this;
        }

        public Configuration idleMode(IdleMode mode) {
            this.idleMode = mode;
            return this;
        }

        public static Configuration mirrorWithCANID(Configuration config, int CANID) {
            return new Configuration(CANID, config.inverted)
                    .currentLimiting(
                            config.enableCurrentLimiting,
                            config.maxFreeSpeedCurrent,
                            config.maxStallCurrent)
                    .voltageCompensation(config.voltageCompensation, config.nominalVoltage)
                    .idleMode(config.idleMode);
        }
    }

    public static final Configuration DEFAULT = new Configuration(0, false);

    private static void handleCANError(int id, REVLibError error, String message) {
        if (error != REVLibError.kOk) {
            DriverStation.reportError(
                    "Could not configure spark id: "
                            + id
                            + " error: "
                            + error.toString()
                            + " "
                            + message,
                    false);
        }
    }

    public static CANSparkMax createSparkMax(Configuration config) {
        CANSparkMax sparkmax = new CANSparkMax(config.CANID, MotorType.kBrushless);
        handleCANError(config.CANID, sparkmax.setCANTimeout(kCANTimeout), "set timeout");
        handleCANError(config.CANID, sparkmax.restoreFactoryDefaults(), "restore factory defaults");
        handleCANError(config.CANID, sparkmax.clearFaults(), "clear faults");

        if (config.enableCurrentLimiting) {
            handleCANError(
                    config.CANID,
                    sparkmax.setSmartCurrentLimit(
                            config.maxStallCurrent, config.maxFreeSpeedCurrent),
                    "set current limiting");
        }

        sparkmax.setInverted(config.inverted);
        handleCANError(config.CANID, sparkmax.setIdleMode(config.idleMode), "set idle mode");

        if (config.voltageCompensation) {
            handleCANError(
                    config.CANID,
                    sparkmax.enableVoltageCompensation(config.nominalVoltage),
                    "set voltage compensation");
        } else {
            handleCANError(
                    config.CANID,
                    sparkmax.disableVoltageCompensation(),
                    "disable voltage compensation");
        }
        return sparkmax;
    }

    public static CANSparkMax createSparkMaxFollower(
            CANSparkMax master, Configuration config, boolean isInvertedFromMaster) {
        CANSparkMax follower = new CANSparkMax(config.CANID, MotorType.kBrushless);
        handleCANError(config.CANID, follower.setCANTimeout(kCANTimeout), "set timeout");
        handleCANError(config.CANID, follower.restoreFactoryDefaults(), "restore factory defaults");
        handleCANError(config.CANID, follower.clearFaults(), "clear faults");
        if (config.voltageCompensation) {
            if (config.voltageCompensation) {
                handleCANError(
                        config.CANID,
                        follower.enableVoltageCompensation(config.nominalVoltage),
                        "set voltage compensation");
            } else {
                handleCANError(
                        config.CANID,
                        follower.disableVoltageCompensation(),
                        "disable voltage compensation");
            }
        }
        handleCANError(config.CANID, follower.follow(master, isInvertedFromMaster), "set follow");
        return follower;
    }

    public static CANSparkMax createSparkMaxFollower(CANSparkMax master, Configuration config) {
        return createSparkMaxFollower(master, config, false);
    }

    public static CANSparkMax createSparkMax() {
        return createSparkMax(DEFAULT);
    }
}
