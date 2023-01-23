package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ManipulatorConstants;

public class ManipulatorSubsystem extends SubsystemBase {
    private final CANSparkMax motor = new CANSparkMax(ManipulatorConstants.kMotorId, MotorType.kBrushless);

    public void intake() {
        motor.set(ManipulatorConstants.kIntakeSpeed);
    }

    public CommandBase release() {
        return Commands.sequence(
            Commands.run(() -> motor.set(ManipulatorConstants.kReleaseSpeed), this), 
            Commands.waitSeconds(ManipulatorConstants.kReleaseTime), 
            stopCommand());
    }

    public void stop() {
        motor.set(0);
    }

    public CommandBase stopCommand() {
        return Commands.run(this::stop, this);
    }

    public boolean motorOverCurrent() {
        return motor.getOutputCurrent() > ManipulatorConstants.kCurrentThreshold;
    }
}
