package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    private final CANSparkMax motor = new CANSparkMax(IntakeConstants.kMotorId, MotorType.kBrushless);
    private final MedianFilter currentFilter = new MedianFilter(10); 
    private double filteredCurrent = 0;

    public IntakeSubsystem() {
        motor.restoreFactoryDefaults();
        motor.setSmartCurrentLimit(IntakeConstants.kCurrentLimit);
        motor.burnFlash();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Current", motor.getOutputCurrent());
        SmartDashboard.putNumber("Intake Median Current", filteredCurrent);
        SmartDashboard.putNumber("Intake Speed", motor.get());
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
        return runOnce(this::stop);
    }

    public boolean motorOverCurrent() {
        filteredCurrent = currentFilter.calculate(motor.getOutputCurrent());
        return filteredCurrent > IntakeConstants.kCurrentThreshold;
    }
}
