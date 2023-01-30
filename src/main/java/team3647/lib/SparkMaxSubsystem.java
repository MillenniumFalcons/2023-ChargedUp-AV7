package team3647.lib;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.List;

public abstract class SparkMaxSubsystem implements PeriodicSubsystem {
    private final CANSparkMax master;
    private final RelativeEncoder encoder;
    private final SparkMaxPIDController controller;
    private final List<CANSparkMax> followers = new ArrayList<>();
    private final double positionConversion;
    private final double velocityConversion;
    private final double nominalVoltage;
    protected final double kDt;

    private PeriodicIO periodicIO = new PeriodicIO();

    public SparkMaxSubsystem(
            CANSparkMax master,
            double positionConversion,
            double velocityConversion,
            double nominalVoltage,
            double kDt) {
        this.master = master;
        this.positionConversion = positionConversion;
        this.velocityConversion = velocityConversion;
        this.nominalVoltage = nominalVoltage;
        this.kDt = kDt;

        this.encoder = master.getEncoder();
        this.controller = master.getPIDController();
    }

    public static class PeriodicIO {
        // Inputs
        public double position = 0;
        public double velocity = 0;
        public double current = 0;
        public double timestamp = 0;
        public double nativePosition = 0;

        // Outputs
        public ControlType controlMode = ControlType.kDutyCycle;
        public double demand = 0;
        public double feedforward = 0;
    }

    @Override
    public void readPeriodicInputs() {
        periodicIO.nativePosition = encoder.getPosition();
        periodicIO.position = periodicIO.nativePosition * velocityConversion;
        periodicIO.velocity = encoder.getVelocity() * velocityConversion;
        periodicIO.current = master.getOutputCurrent();
        periodicIO.timestamp = Timer.getFPGATimestamp();
    }

    @Override
    public void writePeriodicOutputs() {
        controller.setReference(
                periodicIO.demand,
                periodicIO.controlMode,
                0,
                periodicIO.feedforward,
                ArbFFUnits.kVoltage);
    }

    @Override
    public void end() {
        setOpenloop(0);
    }

    public void setOpenloop(double output) {
        periodicIO.controlMode = ControlType.kDutyCycle;
        periodicIO.demand = output;
        periodicIO.feedforward = 0;
    }

    /**
     * Raw PID (not motion magic)
     *
     * @param position in units of positionConvertsion (degrees/meters/etc..)
     * @param feedforward in volts
     */
    protected void setPosition(double position, double feedforward) {
        periodicIO.controlMode = ControlType.kPosition;
        periodicIO.feedforward = feedforward;
        periodicIO.demand = position / positionConversion;
    }

    /**
     * Motion Magic position
     *
     * @param position in units of positionConvertsion (degrees/meters/etc..)
     * @param feedforward in volts
     */
    protected void setPositionMotionMagic(double position, double feedforward) {
        periodicIO.controlMode = ControlType.kSmartMotion;
        periodicIO.feedforward = feedforward;
        periodicIO.demand = position / positionConversion;
    }

    /**
     * @param velocity in the units of velocityConversion (RPM?, Surface speed?)
     * @param feedforward in volts
     */
    protected void setVelocity(double velocity, double feedforward) {
        periodicIO.controlMode = ControlType.kVelocity;
        periodicIO.feedforward = feedforward;
        periodicIO.demand = velocity / velocityConversion;
    }
    /** Sets all motors to brake mode */
    public void setToBrake() {
        setNeutralMode(IdleMode.kCoast);
    }

    /** Sets all motors to coast mode */
    public void setToCoast() {
        setNeutralMode(IdleMode.kCoast);
    }

    /**
     * Sets master and all followers to the mode
     *
     * @param mode either Brake or Coast
     */
    public void setNeutralMode(IdleMode mode) {
        master.setIdleMode(mode);
        for (CANSparkMax follower : followers) {
            follower.setIdleMode(mode);
        }
    }

    /** Sets the selected sensor to 0 (default) */
    public void resetEncoder() {
        setEncoder(0);
    }

    /** Sets the selected sensor to 0 (default) */
    public void resetEncoder(int timeoutMS) {
        setEncoder(0);
    }

    /**
     * sets the selected sensor to position
     *
     * @param position position in output units
     */
    protected void setEncoder(double position) {
        encoder.setPosition(position / positionConversion);
    }

    /**
     * @return the velocity in the output units
     */
    public double getVelocity() {
        return periodicIO.velocity;
    }

    /**
     * @return ths position in the output units
     */
    public double getPosition() {
        return periodicIO.position;
    }

    /**
     * @return the timestamp for the position and velocity measurements
     */
    public double getTimestamp() {
        return periodicIO.timestamp;
    }

    public double getMasterCurrent() {
        return periodicIO.current;
    }

    public double getNativePos() {
        return periodicIO.nativePosition;
    }

    protected void addFollower(CANSparkMax follower, boolean invertFromMaster) {
        follower.follow(master, invertFromMaster);
        followers.add(follower);
    }
}
