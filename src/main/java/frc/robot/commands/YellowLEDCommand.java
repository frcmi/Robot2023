package frc.robot.commands;

import java.util.ArrayList;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.LEDControllerSubsystem;

public class YellowLEDCommand {
    protected final Color8Bit yellow = new Color8Bit(255, 255, 0);
    protected int[] ports;
    protected int[] lengths;
    protected ArrayList<LEDControllerSubsystem> ledControllerArrayList;

    public YellowLEDCommand(int[] ports, int[] lengths) {
        this.lengths = lengths;
        this.ports = ports;
        instantiateControllers();
        execute();
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
    
    public void execute() {
        for (LEDControllerSubsystem ledSub : ledControllerArrayList) {
            ledSub.setColor(yellow);
        }
    }

}
