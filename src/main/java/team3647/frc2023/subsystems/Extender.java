package team3647.frc2023.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import team3647.lib.TalonFXSubsystem;

public class Extender extends TalonFXSubsystem {
    private final SimpleMotorFeedforward feedforward;
    private final double maxLengthTicks;
    private final double minLengthTicks;

    public Extender(
            TalonFX master,
            SimpleMotorFeedforward feedforward,
            double ticksToMetersPerSec,
            double ticksToMeters,
            double minLengthTicks,
            double maxLengthTicks,
            double nominalVoltage,
            double kDt) {
        super(master, ticksToMetersPerSec, ticksToMeters, nominalVoltage, kDt);
        this.feedforward = feedforward;
        this.maxLengthTicks = maxLengthTicks;
        this.minLengthTicks = minLengthTicks;
    }

    @Override
    public void setEncoder(double ticks) {
        super.setEncoder(ticks);
    }

    public void setLengthMeters(double meters) {
        super.setPositionMotionMagic(MathUtil.clamp(meters, minLengthTicks, maxLengthTicks), 0);
    }

    public double getNativeTicks() {
        return super.getNativePos();
    }

    public boolean reachedPosition(double targetPosition, double threshold) {
        return Math.abs(getNativePos() - targetPosition) < threshold;
    }

    @Override
    public String getName() {
        return "Extender";
    }
}
