package team3647.frc2023.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import team3647.lib.TalonFXSubsystem;

public class Extender extends TalonFXSubsystem {
    private final SimpleMotorFeedforward feedforward;

    public Extender(
            TalonFX master,
            SimpleMotorFeedforward feedforward,
            double ticksToMetersPerSec,
            double ticksToMeters,
            double nominalVoltage,
            double kDt) {
        super(master, ticksToMetersPerSec, ticksToMeters, nominalVoltage, kDt);
        this.feedforward = feedforward;
    }

    public void setOpenloop(double percentOut) {
        super.setOpenloop(percentOut);
    }

    public void setLength(double meters) {
        super.setPositionMotionMagic(meters, 0);
    }

    public double getDistance() {
        return super.getPosition();
    }

    @Override
    public String getName() {
        // TODO Auto-generated method stub
        return "Externder";
    }
}
