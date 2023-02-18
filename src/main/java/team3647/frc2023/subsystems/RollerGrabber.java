package team3647.frc2023.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.Solenoid;
import team3647.lib.TalonFXSubsystem;

public class RollerGrabber extends TalonFXSubsystem {
    private final Solenoid piston;

    public RollerGrabber(
            TalonFX motor,
            Solenoid piston,
            double velocityConversion,
            double positionConversion,
            double nominalVoltage,
            double kDt) {
        super(motor, velocityConversion, positionConversion, nominalVoltage, kDt);
        this.piston = piston;
    }

    public void intake() {
        piston.set(false);
        super.setOpenloop(-0.4);
    }

    public void stop() {
        super.setOpenloop(0);
    }

    public void close() {
        stop();
        piston.set(false);
    }

    @Override
    public String getName() {
        // TODO Auto-generated method stub
        return null;
    }
}
