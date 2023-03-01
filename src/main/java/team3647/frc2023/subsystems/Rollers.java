package team3647.frc2023.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import team3647.lib.TalonFXSubsystem;

public class Rollers extends TalonFXSubsystem {

    public Rollers(
            TalonFX motor,
            double velocityConversion,
            double positionConversion,
            double nominalVoltage,
            double kDt) {
        super(motor, velocityConversion, positionConversion, nominalVoltage, kDt);
    }

    public void intake() {
        super.setOpenloop(-1);
    }

    public void outtake() {
        super.setOpenloop(0.5);
    }

    public void stop() {
        super.setOpenloop(0);
    }

    @Override
    public String getName() {
        return "RollerGrabber";
    }
}
