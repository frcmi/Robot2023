package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    private final CANSparkMax motor = new CANSparkMax(IntakeConstants.kMotorId, MotorType.kBrushless);

    public IntakeSubsystem() {
        motor.restoreFactoryDefaults();
        motor.setSmartCurrentLimit(IntakeConstants.kCurrentLimit);
        motor.burnFlash();
    }

    // Intake cone, release cube
    public CommandBase intake() {
        return setMotor(IntakeConstants.kIntakeSpeed)
                .until(this::motorOverCurrent)
                .andThen(Commands.waitSeconds(IntakeConstants.kIntakeTime))
                .andThen(stopCommand());
    }

    private CommandBase setMotor(double speed) {
        return Commands.runOnce(() -> motor.set(speed), this);
    }

    // Release cone, intake cube
    public CommandBase reverseIntake() {
        return setMotor(IntakeConstants.kIntakeSpeed * -1)
                .until(this::motorOverCurrent)
                .andThen(Commands.waitSeconds(IntakeConstants.kIntakeTime))
                .andThen(stopCommand());
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
