package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDControllerSubsystem extends SubsystemBase {
  /** Creates a new LEDSubsystem. */

  protected int stripNumberOfLights;
  protected AddressableLED addressableLED;
  protected AddressableLEDBuffer addressableLEDBuffer;
  protected Color8Bit lightColor;

  public LEDControllerSubsystem(int port, int numberOfLights) {
    addressableLED = new AddressableLED(port);
    stripNumberOfLights = numberOfLights;
    addressableLEDBuffer = new AddressableLEDBuffer(numberOfLights);
    addressableLED.setData(addressableLEDBuffer);
    addressableLED.setLength(addressableLEDBuffer.getLength());
  }

  public void setColor(Color8Bit color) {
    lightColor = color;
    for (int i = 0; i < addressableLEDBuffer.getLength(); i++) {
      addressableLEDBuffer.setLED(i, lightColor);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    addressableLED.start();
  }

}

