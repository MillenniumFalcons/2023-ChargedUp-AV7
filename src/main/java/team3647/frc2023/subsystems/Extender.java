package team3647.frc2023.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import team3647.lib.TalonFXSubsystem;

public class Extender extends TalonFXSubsystem {
    private final SimpleMotorFeedforward feedforward;
    private final double maxLengthMeters;
    private final double minLengthMeters;

    public Extender(
            TalonFX master,
            SimpleMotorFeedforward feedforward,
            double ticksToMetersPerSec,
            double ticksToMeters,
            double minLengthMeters,
            double maxLengthMeters,
            double nominalVoltage,
            double kDt) {
        super(master, ticksToMetersPerSec, ticksToMeters, nominalVoltage, kDt);
        this.feedforward = feedforward;
        this.maxLengthMeters = maxLengthMeters;
        this.minLengthMeters = minLengthMeters;
    }

    public void setEncoder(double ticks) {
        super.setEncoder(ticks);
    }

    public void setOpenloop(double percentOut) {
        super.setOpenloop(percentOut);
    }

    public void setLengthMeters(double meters) {
        super.setPositionMotionMagic(MathUtil.clamp(meters, minLengthMeters, maxLengthMeters), 0);
    }

    public double getLengthMeters() {
        return super.getPosition();
    }

    public double getNativeTicks() {
        return super.getNativePos();
    }

    @Override
    public String getName() {
        // TODO Auto-generated method stub
        return "Externder";
    }
}
