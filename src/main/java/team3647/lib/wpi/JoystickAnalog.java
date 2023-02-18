/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package team3647.lib.wpi;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Add your docs here. */
public class JoystickAnalog extends Trigger {
    double m_threhsold;
    int m_axisNumber;
    GenericHID m_joystick;

    public JoystickAnalog(GenericHID joystick, int axisNumber, double threshold) {
        super(() -> joystick.getRawAxis(axisNumber) > threshold);
        m_threhsold = threshold;
        m_joystick = joystick;
        m_axisNumber = axisNumber;
    }

    /**
     * @return the triggerValue
     */
    public double getValue() {
        return m_joystick.getRawAxis(m_axisNumber);
    }
}
