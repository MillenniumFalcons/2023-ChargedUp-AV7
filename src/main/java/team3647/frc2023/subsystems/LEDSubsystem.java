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
    private boolean target = false;

    private boolean currentCone = false;
    private boolean wantedCone = false;
    private boolean pieceIn = false;

    private Animation cubeColor = LEDConstants.SOLID_PURPLE;
    private Animation coneColor = LEDConstants.SOLID_YELLOW;

    public static enum LEDStates {
        TARGET("TARGET"),
        RAINBOW("RAINBOW"),
        IDLE("IDLE");

        private String name;

        LEDStates(String name) {
            this.name = name;
        }

        public String getLEDState() {
            return name;
        }
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
        this.currentCone = cone;
    }

    public void setWantedPiece(boolean cone) {
        this.wantedCone = cone;
    }

    public void setPieceIn(boolean in) {
        this.pieceIn = in;
    }

    public boolean getPieceIn() {
        return pieceIn;
    }

    public String getLEDState() {
        return wantedState.getLEDState();
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
                break;
            case RAINBOW:
                this.setAnimation(LEDConstants.RAINBOWCONTROLLER);
                break;
            case IDLE:
                if (pieceIn) {
                    if (currentCone) {
                        this.setAnimation(coneColor);
                    } else {
                        this.setAnimation(cubeColor);
                    }
                } else {
                    if (wantedCone) {
                        this.setAnimation(coneColor);
                    } else {
                        this.setAnimation(cubeColor);
                    }
                }
                break;
        }
    }

    @Override
    public String getName() {
        // TODO Auto-generated method stub
        return "LED Subystem";
    }
}
