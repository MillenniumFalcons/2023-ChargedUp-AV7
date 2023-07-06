package team3647.frc2023.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import team3647.lib.TalonFXSubsystem;

public class CubeShooterTop extends TalonFXSubsystem {

    public CubeShooterTop(
            TalonFX master,
            double ticksToMetersPerSec,
            double ticksToMeters,
            double nominalVoltage,
            double kDt) {
        super(master, ticksToMetersPerSec, ticksToMeters, nominalVoltage, kDt);
    }

    public void openLoop(double demand) {
        super.setOpenloop(demand);
    }

    @Override
    public String getName() {
        // TODO Auto-generated method stub
        return "Cube Shooter";
    }
}
