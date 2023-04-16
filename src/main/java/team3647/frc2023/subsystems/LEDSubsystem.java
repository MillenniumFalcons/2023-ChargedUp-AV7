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
    private Animation pieceAnim = LEDConstants.SOLID_PURPLE;

    private Animation targetAnim = LEDConstants.BREATHE_GREEN;
    private Animation currentAnim = pieceAnim;
    private boolean target = false;

    private CANdle m_candle = LEDConstants.m_candle;

    public LEDSubsystem() {
        setAnimation(LEDConstants.BREATHE_RED);
    }

    private void setAnimation(Animation animation) {
        m_candle.animate(animation);
        this.currentAnim = animation;
    }

    public void setPiece(Animation anim) {
        this.pieceAnim = anim;
    }

    public void setTarget(Animation anim) {
        this.targetAnim = anim;
    }

    public void setToPiece() {
        this.target = false;
    }

    public void setToTarget() {
        this.target = true;
    }

    public void setRainbow() {
        this.setAnimation(LEDConstants.RAINBOWCONTROLLER);
    }

    public void setRed() {
        this.setAnimation(LEDConstants.BREATHE_RED);
    }

    @Override
    public void periodic() {
        if (target && currentAnim != this.targetAnim) {
            this.setAnimation(this.targetAnim);
        } else if (!target && currentAnim != this.pieceAnim) {
            this.setAnimation(this.pieceAnim);
        }
    }

    @Override
    public String getName() {
        // TODO Auto-generated method stub
        return "LED Subystem";
    }
}
