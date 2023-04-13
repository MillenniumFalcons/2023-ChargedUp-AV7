// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3647.frc2023.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team3647.frc2023.constants.GlobalConstants;
import team3647.frc2023.constants.LEDConstants;

public class LEDSubsystem extends SubsystemBase {
  /** Creates a new LEDSubsystem. */
  private final CANdle m_candle = new CANdle(LEDConstants.CANdleID, "drive");

  private static final LEDSubsystem candleSub = new LEDSubsystem();
  private Animation animation = null;

  public static LEDSubsystem getInstance() {
      return candleSub;
  }
  public LEDSubsystem() {
    setAnimation(LEDConstants.BREATHE_RED);
  }

  public void setAnimation(Animation animation) {
    this.animation = animation;
    m_candle.animate(this.animation);
    System.out.println("Setting LEDs");
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
