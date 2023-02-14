package team3647.frc2023.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Grabber extends SubsystemBase {
    private final Solenoid pistons;

    public Grabber(Solenoid pistons) {
        this.pistons = pistons;
        close();
    }

    public void open() {
        pistons.set(true);
    }

    public void close() {
        pistons.set(false);
    }
}
