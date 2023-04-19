package team3647.frc2023.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.playingwithfusion.TimeOfFlight;
import edu.wpi.first.math.MathUtil;
import team3647.lib.TalonFXSubsystem;

public class CubeWrist extends TalonFXSubsystem {
    private double minDegree;
    private double maxDegree;
    private double kG;
    private TimeOfFlight tof;
    private final double triggerDistance = 600;

    private double sensorDistance = 1000;

    public CubeWrist(
            TalonFX master,
            TimeOfFlight tof,
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
        this.tof = tof;
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
    public void readPeriodicInputs() {
        super.readPeriodicInputs();
        this.sensorDistance = this.tof.getRange();
    }

    public double getTofDist() {
        return this.sensorDistance;
    }

    public boolean isSensorTriggered() {
        return getTofDist() < triggerDistance;
    }

    @Override
    public String getName() {
        return "Cube Wrist";
    }
}
