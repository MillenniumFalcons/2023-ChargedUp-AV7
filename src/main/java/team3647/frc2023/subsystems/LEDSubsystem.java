// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3647.frc2023.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import team3647.frc2023.constants.LEDConstants;
import team3647.lib.PeriodicSubsystem;

public class LEDSubsystem implements PeriodicSubsystem {
    /** Creates a new LEDSubsystem. */
    private Animation targetAnim = LEDConstants.BREATHE_GREEN;

    private boolean target = false;
    private boolean cone = false;

    public static enum LEDStates {
        TARGET,
        RAINBOW,
        IDLE
    }

    private LEDStates currentState;
    private LEDStates wantedState = LEDStates.IDLE;

    private CANdle m_candle = LEDConstants.m_candle;

    public LEDSubsystem() {
        setAnimation(LEDConstants.BREATHE_RED);
    }

    private void setAnimation(Animation animation) {
        m_candle.animate(animation);
    }

    public void setLEDState(LEDStates state) {
        wantedState = state;
    }

    public void setTarget(boolean bool) {
        this.target = bool;
    }

    public void setPiece(boolean cone) {
        this.cone = cone;
    }

    @Override
    public void periodic() {
        switch (wantedState) {
            case TARGET:
                if (target) {
                    this.setAnimation(LEDConstants.BREATHE_GREEN);
                } else {
                    this.setAnimation(LEDConstants.BREATHE_RED);
                }
            case RAINBOW:
                this.setAnimation(LEDConstants.RAINBOWCONTROLLER);
            case IDLE:
                if (cone) {
                    this.setAnimation(LEDConstants.SOLID_YELLOW);
                } else {
                    this.setAnimation(LEDConstants.SOLID_PURPLE);
                }
        }
    }

    @Override
    public String getName() {
        // TODO Auto-generated method stub
        return "LED Subystem";
    }
}
