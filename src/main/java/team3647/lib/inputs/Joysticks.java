package team3647.lib.inputs;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Joysticks wrapper to provide easier access to buttons, triggers and sticks. */
public class Joysticks {

    public final Trigger leftTrigger;
    public final Trigger rightTrigger;

    public final Trigger rightJoyStickPress;
    public final Trigger leftJoyStickPress;
    public final Trigger leftMidButton;
    public final Trigger rightMidButton;

    public final Trigger rightBumper;
    public final Trigger leftBumper;
    public final Trigger buttonA;
    public final Trigger buttonB;
    public final Trigger buttonY;
    public final Trigger buttonX;
    public final Trigger dPadDown;
    public final Trigger dPadLeft;
    public final Trigger dPadRight;
    public final Trigger dPadUp;

    public final Trigger rightStickMoved;

    /** XboxController Object for Controller; contains all Xbox Controller Functions */
    private final CommandXboxController controller;

    private final int controllerPin;

    public Joysticks(int controllerPin) {
        this.controllerPin = controllerPin;
        controller = new CommandXboxController(controllerPin);

        leftTrigger = controller.leftTrigger(0.5);
        rightTrigger = controller.rightTrigger(0.15);

        rightJoyStickPress = controller.rightStick();
        leftJoyStickPress = controller.leftStick();
        leftMidButton = controller.back();
        rightMidButton = controller.start();

        rightBumper = controller.rightBumper();
        leftBumper = controller.leftBumper();
        buttonA = controller.a();
        buttonB = controller.b();
        buttonY = controller.y();
        buttonX = controller.x();

        dPadDown = controller.povDown();
        dPadLeft = controller.povLeft();
        dPadRight = controller.povRight();
        dPadUp = controller.povUp();

        rightStickMoved = new Trigger(this::rightStickMoved);
    }

    public double getLeftStickX() {
        return applyDeadband(controller.getLeftX());
    }

    public double getLeftStickY() {
        return applyDeadband(-controller.getLeftY());
    }

    public double getRightStickX() {
        return applyDeadband(controller.getRightX());
    }

    public double getRightStickY() {
        return applyDeadband(-controller.getRightY());
    }

    public double getRightTriggerValue() {
        return applyDeadband(controller.getRightTriggerAxis());
    }

    public double getLeftTriggerValue() {
        return applyDeadband(controller.getLeftTriggerAxis());
    }

    public boolean anyStickMoved() {
        return Math.abs(getLeftStickX()) > 0.15
                || Math.abs(getLeftStickY()) > 0.15
                || Math.abs(getRightStickX()) > 0.15;
    }

    public boolean anyStickMovedFast() {
        return Math.abs(getLeftStickX()) > 0.5
                || Math.abs(getLeftStickY()) > 0.5
                || Math.abs(getRightStickX()) > 0.5;
    }

    public boolean rightStickMoved() {
        return Math.abs(getRightStickX()) > 0.1 || Math.abs(getRightStickY()) > 0.1;
    }

    public boolean anyStickMovedStiff() {
        return Math.abs(getLeftStickX()) > 0.15
                || Math.abs(getLeftStickY()) > 0.15
                || Math.abs(getRightStickX()) > 0.15;
    }
    /**
     * Returns 0.0 if the given value is within the specified range around zero. The remaining range
     * between the deadband and 1.0 is scaled from 0.0 to 1.0.
     *
     * @param value value to clip
     * @param deadband range around zero
     */
    private double applyDeadband(double value, double deadband) {
        if (Math.abs(value) < deadband) {
            return 0.0;
        }

        if (value > 0.0) {
            return (value - deadband) / (1.0 - deadband);
        }
        return (value + deadband) / (1.0 - deadband);
    }

    /**
     * Returns 0.0 if the given value is within the specified range around zero. The remaining range
     * between the deadband and 1.0 is scaled from 0.0 to 1.0.
     *
     * @param value value to clip
     * @param deadband range around zero
     */
    private double applyDeadband(double value) {
        return applyDeadband(value, 0.09);
    }

    public int getControllerPin() {
        return controllerPin;
    }
}
