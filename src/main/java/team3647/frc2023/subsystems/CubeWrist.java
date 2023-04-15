package team3647.frc2023.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.MathUtil;
import team3647.lib.TalonFXSubsystem;

public class CubeWrist extends TalonFXSubsystem {
    private double minDegree;
    private double maxDegree;
    private double kG;

    public CubeWrist(
            TalonFX master,
            double ticksToDegPerSecConversion,
            double ticksToDegConversion,
            double nominalVoltage,
            double kG,
            double minDegree,
            double maxDegree,
            double kDt) {
        super(master, ticksToDegPerSecConversion, ticksToDegConversion, nominalVoltage, kDt);
        this.minDegree = minDegree;
        this.maxDegree = maxDegree;
        this.kG = kG;
    }

    @Override
    public void setEncoder(double degree) {
        super.setEncoder(degree);
    }

    public void setAngle(double angle) {
        super.setPositionMotionMagic(MathUtil.clamp(angle, minDegree, maxDegree), 0);
    }

    public double getAngle() {
        return super.getPosition();
    }

    public boolean angleReached(double targetAngle, double threshold) {
        return Math.abs(getAngle() - targetAngle) < threshold;
    }

    @Override
    public String getName() {
        // TODO Auto-generated method stub
        return "Cube Wrist";
    }
}
