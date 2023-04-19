package team3647.frc2023.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import team3647.lib.TalonFXSubsystem;

public class Rollers extends TalonFXSubsystem {
    private final DigitalInput cubeSensor;

    private boolean hasCube = false;
    private Timer gamePiecetimer = new Timer();
    private final double hasCubeWaitTime = 0.5;
    private final double cubeCurrent = 10;

    public Rollers(
            TalonFX motor,
            DigitalInput cubeSensor,
            double velocityConversion,
            double positionConversion,
            double nominalVoltage,
            double kDt) {
        super(motor, velocityConversion, positionConversion, nominalVoltage, kDt);
        this.cubeSensor = cubeSensor;
    }

    public void intakeCone() {
        super.setOpenloop(-1);
    }

    public void intakeCube() {
        super.setOpenloop(0.5);
    }

    public void intakeGround() {
        super.setOpenloop(-0.5);
    }

    public void outtakeCone() {
        super.setOpenloop(0.35);
    }

    public void outtakeCube() {
        super.setOpenloop(-0.2);
    }

    public void stop() {
        super.setOpenloop(0);
    }

    private boolean getCubeSensorStatus() {
        return !cubeSensor.get();
    }

    public boolean hasCube() {
        return hasCube;
    }

    @Override
    public String getName() {
        return "RollerGrabber";
    }
}
