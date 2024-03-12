// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {

  private AddressableLED leds;
  private AddressableLEDBuffer patternBuffer;

  /** Creates a new LEDSubsystem. */
  public LEDSubsystem() {
    // PWM port 9
    // Must be a PWM header, not MXP or DIO
    leds = new AddressableLED(9);

    // Default to start with empty output
    // Length is expensive to set, so only set it once, then just update data
    patternBuffer = new AddressableLEDBuffer(20);
    leds.setLength(patternBuffer.getLength());

    // Set the data
    leds.setData(patternBuffer);
    leds.start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    leds.setData(patternBuffer);
  }
}
