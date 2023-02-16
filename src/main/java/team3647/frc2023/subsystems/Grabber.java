package team3647.frc2023.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team3647.lib.NetworkColorSensor;
import team3647.lib.NetworkColorSensor.GamePiece;

public class Grabber extends SubsystemBase {
    private final Solenoid pistons;
    private final NetworkColorSensor colorSensor;

    public Grabber(Solenoid pistons, NetworkColorSensor colorSensor) {
        this.pistons = pistons;
        this.colorSensor = colorSensor;
        close();
    }

    public void close() {
        pistons.set(false);
    }

    public void open() {
        pistons.set(true);
    }

    public GamePiece getGamepiece() {
        // return colorSensor.getGamepiece();
        return GamePiece.CONE;
    }

    public String getGamePieceStr() {
        return colorSensor.getGamepiece().str;
    }

    @Override
    public String getName() {
        // TODO Auto-generated method stub
        return "Grabber";
    }
}
