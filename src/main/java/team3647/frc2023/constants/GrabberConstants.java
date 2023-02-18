package team3647.frc2023.constants;

import edu.wpi.first.wpilibj.Solenoid;

public class GrabberConstants {
    public static final Solenoid pistons =
            new Solenoid(GlobalConstants.kPCMType, GlobalConstants.GrabberIds.pistonChannel);

    static {
    }
}
