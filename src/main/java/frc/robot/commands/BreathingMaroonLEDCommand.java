package frc.robot.commands;

import java.util.ArrayList;

import java.awt.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.LEDControllerSubsystem;

public class BreathingMaroonLEDCommand {
    protected final Color8Bit initialMaroon = new Color8Bit(144, 56, 32);
    protected Color8Bit currentColor;
    protected int[] ports;
    protected int[] lengths;
    protected boolean increasing = true;
    protected boolean breathing;
    protected ArrayList<LEDControllerSubsystem> ledControllerArrayList;

    public BreathingMaroonLEDCommand(int[] ports, int[] lengths) {
        this.lengths = lengths;
        this.ports = ports;
        currentColor = initialMaroon;
        breathing = false;
    }

    public void instantiateControllers() {
        int i = 0;
        try {
            for (int port : ports) {
                ledControllerArrayList.add(new LEDControllerSubsystem(port, lengths[i]));
                i++;
            }
        } catch (ArrayIndexOutOfBoundsException e) {
            // TODO: handle exception
            System.out.println("Length array and port array sizes don't match up");
        }
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

    public void periodic() {
        if (breathing) {
            currentColor = breathingColor();
        }
        for (LEDControllerSubsystem ledSub : ledControllerArrayList) {
            ledSub.setColor(currentColor);
        }
    }

    public boolean getBreathing() {
        return breathing;
    }

    public void setBreathing(boolean breathTime) {
        breathing = breathTime;
    }

}
