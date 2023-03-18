package team3647.frc2023.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import team3647.lib.TalonFXSubsystem;

public class Wrist extends TalonFXSubsystem {
    private final double minDegree;
    private final double maxDegree;
    private final double kG;

    public Wrist(
            TalonFX master,
            double ticksToDegsPerSec,
            double ticksToDegs,
            double minDegree,
            double maxDegree,
            double nominalVoltage,
            double kG,
            double kDt) {
        super(master, ticksToDegsPerSec, ticksToDegs, nominalVoltage, kDt);
        this.minDegree = minDegree;
        this.maxDegree = maxDegree;
        this.kG = kG;

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
        SmartDashboard.putNumber("Wrist set", angle);
        var ffVolts = kG * Math.cos(Units.degreesToRadians(angle));
        SmartDashboard.putNumber("WRIST FF", ffVolts);
        this.setPositionMotionMagic(angle, ffVolts);
    }

    public boolean angleReached(double targetAngle, double threshold) {
        return Math.abs(getAngle() - targetAngle) < threshold;
    }

    @Override
    public String getName() {
        return "Wrist";
    }
}
