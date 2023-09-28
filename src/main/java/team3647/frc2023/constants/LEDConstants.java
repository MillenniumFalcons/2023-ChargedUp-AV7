// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3647.frc2023.constants;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;

/** Add your docs here. */
public class LEDConstants {
    public static final int CANdleID = 17;
    public static final CANdle m_candle = new CANdle(LEDConstants.CANdleID, "drive");

    // LED Counts
    public static final int candleLEDS = 8;
    public static final int stripLEDS = 144;
    public static final int LEDCOUNT = candleLEDS + 2 * stripLEDS;

    // Animations List
    // Rainbows
    public static final Animation RAINBOW = new RainbowAnimation(1, 0.1, LEDCOUNT);
    public static final Animation RAINBOWCONTROLLER = new RainbowAnimation(1, 0.5, LEDCOUNT);

    // Unused
    public static final Animation GREEN_STROBE =
            new StrobeAnimation(0, 255, 0, 0, 56.0 / 256.0, LEDCOUNT);
    public static final Animation LARSON =
            new LarsonAnimation(255, 0, 0, 0, 0.75, LEDCOUNT, BounceMode.Front, 50);
    public static final Animation COLOR_FLOW =
            new ColorFlowAnimation(255, 0, 0, 0, 0.7, LEDCOUNT, Direction.Forward);

    // Solid Colors
    public static final Animation SOLID_YELLOW = new StrobeAnimation(246, 190, 0, 128, 1, LEDCOUNT);
    public static final Animation SOLID_PURPLE =
            new StrobeAnimation(162, 25, 255, 128, 1, LEDCOUNT);
    public static final Animation SOLID_PINK = new StrobeAnimation(255, 0, 255, 128, 1, LEDCOUNT);
    public static final Animation SOLID_WHITE =
            new StrobeAnimation(255, 255, 255, 128, 1, LEDCOUNT);

    // Breathing/Flashing Animations
    public static final Animation BREATHE_RED =
            new SingleFadeAnimation(255, 0, 0, 0, 0.8, LEDCOUNT);
    public static final Animation BREATHE_GREEN =
            new SingleFadeAnimation(0, 255, 0, 0, 0.65, LEDCOUNT);
    public static final Animation FLASH_PURPLE =
            new StrobeAnimation(162, 25, 255, 128, 0.3, LEDCOUNT);
}
