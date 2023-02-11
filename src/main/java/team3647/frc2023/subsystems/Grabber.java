package team3647.frc2023.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import team3647.lib.NetworkColorSensor;
import team3647.lib.NetworkColorSensor.GamePiece;
import team3647.lib.TalonFXSubsystem;

public class Grabber extends TalonFXSubsystem {
    private final SimpleMotorFeedforward feedforward;
    private final double ticksToDegsPerSec;
    private final NetworkColorSensor colorSensor;

    public Grabber(
            TalonFX master,
            SimpleMotorFeedforward feedforward,
            double ticksToDegsPerSec,
            double ticksToDegs,
            double nominalVoltage,
            double kDt,
            NetworkColorSensor colorSenor) {
        super(master, ticksToDegsPerSec, ticksToDegs, nominalVoltage, kDt);
        this.feedforward = feedforward;
        this.ticksToDegsPerSec = ticksToDegsPerSec;
        super.resetEncoder();
        this.colorSensor = colorSenor;
    }

    public void setAngle(double degrees) {
        // super.setPositionMotionMagic(degrees, feedforward.calculate(6000.0 / ticksToDegsPerSec));
        super.setPosition(degrees, 0);
    }

    public boolean isClosed() {
        return super.getPosition() > 155;
    }

    public void setOpenloop(double demand) {
        super.setOpenloop(demand);
    }

    public double getAngle() {
        return super.getPosition();
    }

    public GamePiece getGamepiece() {
        return colorSensor.getGamepiece();
    }

    @Override
    public String getName() {
        // TODO Auto-generated method stub
        return "Grabber";
    }
}
