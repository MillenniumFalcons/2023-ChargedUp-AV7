package team3647.frc2023.subsystems;

import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.function.DoubleSupplier;
import team3647.lib.TalonFXSubsystem;

public class Pivot extends TalonFXSubsystem {
    private final DoubleSupplier getKG;
    private final double minDegree;
    private final double maxDegree;

    public Pivot(
            TalonFX master,
            TalonFX slave,
            double ticksToDegsPerSec,
            double ticksToDegs,
            double minDegree,
            double maxDegree,
            double nominalVoltage,
            DoubleSupplier getKGFromExtender,
            double kDt) {
        super(master, ticksToDegsPerSec, ticksToDegs, nominalVoltage, kDt);
        super.addFollower(slave, FollowerType.PercentOutput, InvertType.FollowMaster);
        this.getKG = getKGFromExtender;
        this.minDegree = minDegree;
        this.maxDegree = maxDegree;
    }

    public void setEncoder(double degree) {
        super.setEncoder(degree);
    }

    public void setOpenloop(double percentOut) {
        super.setOpenloop(percentOut);
    }

    public void setAngle(double angle) {
        var ffVolts = getKG.getAsDouble() * Math.cos(Units.degreesToRadians(angle));
        super.setPositionMotionMagic(MathUtil.clamp(angle, minDegree, maxDegree), 0);
        SmartDashboard.putNumber("Pivot ff volts", ffVolts);
    }

    public double getAngle() {
        return super.getPosition();
    }

    @Override
    public String getName() {
        return "Pivot";
    }
}
