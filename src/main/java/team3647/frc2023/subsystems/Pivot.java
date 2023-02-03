package team3647.frc2023.subsystems;

import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import team3647.lib.TalonFXSubsystem;

public class Pivot extends TalonFXSubsystem {
    private final double kG;

    public Pivot(
            TalonFX master,
            TalonFX slave,
            double ticksToDegsPerSec,
            double ticksToDegs,
            double nominalVoltage,
            double kG,
            double kDt) {
        super(master, ticksToDegsPerSec, ticksToDegs, nominalVoltage, kDt);
        super.addFollower(slave, FollowerType.PercentOutput, InvertType.FollowMaster);
        this.kG = kG;
        super.resetEncoder();
    }

    public void setOpenloop(double percentOut) {
        super.setOpenloop(percentOut);
    }

    public void setAngle(double angle, double feedforward) {
        super.setPositionMotionMagic(angle, kG);
        SmartDashboard.putNumber("KG", feedforward);
    }

    public double getAngle() {
        return super.getPosition();
    }

    @Override
    public String getName() {
        // TODO Auto-generated method stub
        return "Pivot";
    }
}
