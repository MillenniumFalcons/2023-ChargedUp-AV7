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
        Column9(12),
        Red1(8),
        Red2(5),
        Red3(9),
        Red4(4),
        White1(7),
        White2(6);

        public final int value;

        Buttons(int index) {
            this.value = index;
        }
    }

    public ControlPanel(int pin) {
        this.pin = pin;
    }

    public boolean getLevelLow() {
        return getButton(Buttons.LevelLow);
    }

    public boolean getLevelMid() {
        return getButton(Buttons.LevelMid);
    }

    public boolean getLevelHigh() {
        return getButton(Buttons.LevelHigh);
    }

    public boolean getColumnOne() {
        return getButton(Buttons.Column1);
    }

    public boolean getColumnTwo() {
        return getButton(Buttons.Column2);
    }

    public boolean getColumnThree() {
        return getButton(Buttons.Column3);
    }

    public boolean getColumnFour() {
        return getButton(Buttons.Column4);
    }

    public boolean getColumnFive() {
        return getButton(Buttons.Column5);
    }

    public boolean getColumnSix() {
        return getButton(Buttons.Column6);
    }

    public boolean getColumnSeven() {
        return getButton(Buttons.Column7);
    }

    public boolean getColumnEight() {
        return getButton(Buttons.Column8);
    }

    public boolean getColumnNine() {
        return getButton(Buttons.Column9);
    }

    public boolean getRedTwo() {
        return getButton(Buttons.Red2);
    }

    public boolean getWhiteTwo() {
        return getButton(Buttons.White2);
    }

    public boolean getRedThree() {
        return getButton(Buttons.Red3);
    }

    public boolean getWhiteOne() {
        return getButton(Buttons.White1);
    }

    public boolean getRedOne() {
        return getButton(Buttons.Red1);
    }

    public boolean getRedFour() {
        return getButton(Buttons.Red4);
    }

    public double getJoystickFBAxis() {
        return DriverStation.getStickAxis(this.pin, 0);
    }

    public boolean getButton(Buttons button) {
        return DriverStation.getStickButton(this.pin, button.value);
    }
}
