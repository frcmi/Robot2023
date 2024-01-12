package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SparkMax;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    //private final CANSparkMax motor = new CANSparkMax(IntakeConstants.kMotorId, MotorType.kBrushless);
    private final SparkMax motor = new SparkMax(IntakeConstants.kMotorId, MotorType.kBrushless);
    private final MedianFilter currentFilter = new MedianFilter(10); 
    private double filteredCurrent = 0;
    
    public IntakeSubsystem() {
        //motor.restoreFactoryDefaults();
        motor.setSmartCurrentLimit(30, 40);
        //motor.burnFlash();
        motor.setIdleMode(IdleMode.kBrake);
        setDefaultCommand(stopCommand());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Current", motor.getOutputCurrent());
        SmartDashboard.putNumber("Intake Median Current", filteredCurrent);
        SmartDashboard.putNumber("Intake Speed", motor.get());
        var currentCommand = this.getCurrentCommand();
        if (currentCommand != null)
        SmartDashboard.putString("Intake Command", currentCommand.getName());
    }

    // Intake cone, release cube
    public Command intake() {
        return setMotor(IntakeConstants.kIntakeSpeed)
                // .until(this::motorOverCurrent)
                // .andThen(Commands.waitSeconds(IntakeConstants.kIntakeTime))
                // .andThen(stopCommand())
                ;
    }

    private Command setMotor(double speed) {
        return Commands.run(() -> motor.set(speed), this);
    }

    // Release cone, intake cube
    public Command reverseIntake() {
        return setMotor(IntakeConstants.kIntakeSpeed * -1)
                // .until(this::motorOverCurrent)
                // .andThen(Commands.waitSeconds(IntakeConstants.kIntakeTime))
                // .andThen(stopCommand())
                ;
    }

    public void stop() {
        motor.set(0);
    }

    public Command stopCommand() {
        return runOnce(this::stop);
    }

    public boolean motorOverCurrent() {
        filteredCurrent = currentFilter.calculate(motor.getOutputCurrent());
        return filteredCurrent > IntakeConstants.kCurrentThreshold;
    }
}
