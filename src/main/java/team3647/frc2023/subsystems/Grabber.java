package team3647.frc2023.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import team3647.lib.TalonFXSubsystem;

public class Grabber extends TalonFXSubsystem {
    private final SimpleMotorFeedforward feedforward;
    private final double ticksToDegsPerSec;

    public Grabber(
            TalonFX master,
            SimpleMotorFeedforward feedforward,
            double ticksToDegsPerSec,
            double ticksToDegs,
            double nominalVoltage,
            double kDt) {
        super(master, ticksToDegsPerSec, ticksToDegs, nominalVoltage, kDt);
        this.feedforward = feedforward;
        this.ticksToDegsPerSec = ticksToDegsPerSec;
        super.resetEncoder();
    }

    public void setAngle(double degrees) {
        super.setPositionMotionMagic(degrees, feedforward.calculate(100 / ticksToDegsPerSec));
        // super.setPosition(degrees, feedforward.calculate(90 / ticksToDegsPerSec));
    }

    public void setOpenloop(double demand) {
        super.setOpenloop(demand);
    }

    public double getAngle() {
        return super.getPosition();
    }

    @Override
    public String getName() {
        // TODO Auto-generated method stub
        return "Grabber";
    }
}
