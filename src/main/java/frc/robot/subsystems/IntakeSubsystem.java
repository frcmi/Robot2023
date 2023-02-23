package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    private final CANSparkMax motor = new CANSparkMax(IntakeConstants.kMotorId, MotorType.kBrushless);

    public CommandBase intake() {
        return setMotor(IntakeConstants.kIntakeSpeed)
                .until(this::motorOverCurrent)
                .andThen(stopCommand());
    }

    private CommandBase setMotor(double speed) {
        return Commands.runOnce(() -> motor.set(speed), this);
    }

    public CommandBase release() {
        return Commands.sequence(
            setMotor(IntakeConstants.kReleaseSpeed),
            Commands.waitSeconds(IntakeConstants.kReleaseTime),
            stopCommand());
    }

    public void stop() {
        motor.set(0);
    }

    public CommandBase stopCommand() {
        return Commands.runOnce(this::stop, this);
    }

    public boolean motorOverCurrent() {
        return motor.getOutputCurrent() > IntakeConstants.kCurrentThreshold;
    }
}
