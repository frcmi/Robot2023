// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MiscellaneousConstants;
import frc.robot.Constants.ColorConstants;

public class LEDSubsystem extends SubsystemBase {
  /** Creates a new LEDSubsystem. */

  protected int stripNumberOfLights;
  protected AddressableLED addressableLED;
  protected AddressableLEDBuffer addressableLEDBuffer;
  protected Color8Bit lightColor;

  public LEDSubsystem(int numberOfLights, Color8Bit color) {
    addressableLED = new AddressableLED(MiscellaneousConstants.kLightPWMPort);
    stripNumberOfLights = numberOfLights;
    lightColor = color;
    addressableLEDBuffer = new AddressableLEDBuffer(numberOfLights);
    addressableLED.setData(addressableLEDBuffer);
    addressableLED.setLength(addressableLEDBuffer.getLength());
    setColor(lightColor);
  }

  public void setColor(Color8Bit color) {
    for (int i = 0; i < addressableLEDBuffer.getLength(); i++) {
      addressableLEDBuffer.setLED(i, lightColor);
    }
  }
  public void setPurple() {
    setColor(ColorConstants.kPurple);
  }

  public void setYellow() {
    setColor(ColorConstants.kYellow);
  }

  public int getNumberOfLights() {
    return stripNumberOfLights;
  }

  public Color8Bit getLightColor() {
    return lightColor;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    addressableLED.start();
  }

}
