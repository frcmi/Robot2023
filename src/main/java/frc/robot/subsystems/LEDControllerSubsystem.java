package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.awt.Color;
import frc.robot.Constants.LEDConstants;

public class LEDControllerSubsystem extends SubsystemBase {
  /** Creates a new LEDSubsystem. */

  protected Color8Bit currentColor;
  protected AddressableLED addressableLED;
  protected AddressableLEDBuffer addressableLEDBuffer;
  protected boolean increasing = true;
  protected boolean breathing;
  protected boolean ledColorState = true;

  public LEDControllerSubsystem() {
    addressableLED = new AddressableLED(LEDConstants.kLightPort);
    addressableLEDBuffer = new AddressableLEDBuffer(LEDConstants.kNumberOfLights);
    addressableLED.setLength(addressableLEDBuffer.getLength());

    addressableLED.setData(addressableLEDBuffer);
    addressableLED.start();
  }

  public CommandBase setColor(Color8Bit color) {
    currentColor = color;
    return runOnce(() -> {
      for (int i = 0; i < addressableLEDBuffer.getLength(); i++) {
        addressableLEDBuffer.setLED(i, color);
      }
    });
  }

  public CommandBase startBreathing() {
    return runOnce(() -> breathing = true);
  }

  public CommandBase stopBreathing() {
    return runOnce(() -> breathing = false);
  }

  public CommandBase yellowLEDCommand() {
    return stopBreathing().andThen(setColor(LEDConstants.kYellow));
  }

  public CommandBase purpleLEDCommand() {
    return stopBreathing().andThen(setColor(LEDConstants.kPurple));
  }

  public CommandBase maroonLEDCommand() {
    return startBreathing().andThen(setColor(LEDConstants.kInitialMaroon));
  }

  public CommandBase toggleLEDCommand() {
    return runOnce(() -> {
      if (ledColorState) {
        ledColorState = false;
        stopBreathing().andThen(setColor(LEDConstants.kPurple)).schedule();;
      }
      else {
        ledColorState = true;
        stopBreathing().andThen(setColor(LEDConstants.kYellow)).schedule();;
      }
    });
  }

  public Color8Bit breathingColor() {
    float[] HSBArray = Color.RGBtoHSB(currentColor.red, currentColor.green, currentColor.blue, null);
    if (increasing) {
        HSBArray[2] += 1.0;
    } else {
        HSBArray[2] -= 1.0;
    }
    if (HSBArray[2] > 80.0 && increasing == true) {
        increasing = false;
    } else if (HSBArray[2] < 20.0 && increasing == false) {
        increasing = true;
    }
    int RGBOneDigit = Color.HSBtoRGB(HSBArray[0], HSBArray[1], HSBArray[2]);
    Color tempColor = new Color(RGBOneDigit);
    return new Color8Bit(tempColor.getRed(), tempColor.getGreen(), tempColor.getBlue());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("COLOR TOGGLESTATE", ledColorState);
    if (breathing) {
      currentColor = breathingColor();
      setColor(currentColor);
    }
    // addressableLED.start();
    addressableLED.setData(addressableLEDBuffer);
  }
}

