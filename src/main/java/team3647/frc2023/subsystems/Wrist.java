package team3647.frc2023.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import team3647.lib.TalonFXSubsystem;

public class Wrist extends TalonFXSubsystem {
    private final double minDegree;
    private final double maxDegree;

    public Wrist(
            TalonFX master,
            double ticksToDegsPerSec,
            double ticksToDegs,
            double minDegree,
            double maxDegree,
            double nominalVoltage,
            double kDt) {
        super(master, ticksToDegsPerSec, ticksToDegs, nominalVoltage, kDt);
        this.minDegree = minDegree;
        this.maxDegree = maxDegree;
        // setToBrake();
    }

    @Override
    public void setEncoder(double degree) {
        super.setEncoder(degree);
    }

    public double getAngle() {
        return super.getPosition();
    }

    public void setAngle(double angle) {
        SmartDashboard.putNumber("setting angle", angle);
        this.setPositionMotionMagic(angle, 0);
    }

    public boolean angleReached(double targetAngle, double threshold) {
        return Math.abs(getAngle() - targetAngle) < threshold;
    }

    @Override
    public String getName() {
        return "Wrist";
    }
}
