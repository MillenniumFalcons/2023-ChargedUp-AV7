package team3647.lib.inputs;

import edu.wpi.first.wpilibj.DriverStation;

public class ControlPanel {
    private final int pin;

    public enum Buttons {
        LevelLow(1),
        LevelMid(2),
        LevelHigh(3),
        Column1(4),
        Column2(5),
        Column3(6),
        Column4(7),
        Column5(8),
        Column6(9),
        Column7(10),
        Column8(11),
        Column9(12);

        public final int value;

        Buttons(int index) {
            this.value = index;
        }
    }

    public ControlPanel(int pin) {
        this.pin = pin;
    }

    public boolean getButton(Buttons button) {
        return DriverStation.getStickButton(this.pin, button.value);
    }
}
