package team3647.frc2023.subsystems;

import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.controller.ArmFeedforward;
import java.util.function.DoubleSupplier;
import team3647.lib.TalonFXSubsystem;

public class Pivot extends TalonFXSubsystem {
    private final DoubleSupplier armGetLength;
    private final ArmFeedforward feedforward;

    public Pivot(
            TalonFX master,
            TalonFX slave,
            DoubleSupplier armGetLength,
            ArmFeedforward feedforward,
            double ticksToDegsPerSec,
            double ticksToDegs,
            double nominalVoltage,
            double kDt) {
        super(master, ticksToDegsPerSec, ticksToDegs, nominalVoltage, kDt);
        super.addFollower(slave, FollowerType.PercentOutput, InvertType.FollowMaster);
        this.armGetLength = armGetLength;
        this.feedforward = feedforward;
    }

    public void setAngle(double angle) {
        super.setPositionMotionMagic(angle, 0);
    }

    @Override
    public String getName() {
        // TODO Auto-generated method stub
        return "Pivot";
    }
}
