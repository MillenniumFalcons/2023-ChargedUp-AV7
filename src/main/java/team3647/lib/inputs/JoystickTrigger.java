/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package team3647.lib.inputs;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Add your docs here. */
public class JoystickTrigger extends Trigger {
    double mthrehsold;
    int maxisNumber;
    GenericHID mjoystick;

    public JoystickTrigger(GenericHID joystick, int axisNumber, double threshold) {
        super(() -> joystick.getRawAxis(axisNumber) > threshold);
        mthrehsold = threshold;
        mjoystick = joystick;
        maxisNumber = axisNumber;
    }

    /**
     * @return the triggerValue
     */
    public double getTriggerValue() {
        return mjoystick.getRawAxis(maxisNumber);
    }
}
