package team3647.frc2023.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.controller.ArmFeedforward;
import java.util.function.DoubleSupplier;
import team3647.lib.TalonFXSubsystem;

public class Extender extends TalonFXSubsystem {

    public Extender(
            TalonFX master,
            DoubleSupplier armGetLength,
            ArmFeedforward feedforward,
            double ticksToDegsPerSec,
            double ticksToDegs,
            double nominalVoltage,
            double kDt) {
        super(master, ticksToDegsPerSec, ticksToDegs, nominalVoltage, kDt);
    }

    public void setLength(double meters) {
        super.setPositionMotionMagic(meters, 0);
    }

    @Override
    public String getName() {
        // TODO Auto-generated method stub
        return "Externder";
    }
}
