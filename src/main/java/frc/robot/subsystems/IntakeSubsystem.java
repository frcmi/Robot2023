package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SparkMax;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LoggingConfig;

public class IntakeSubsystem extends SubsystemBase {
    //private final CANSparkMax motor = new CANSparkMax(IntakeConstants.kMotorId, MotorType.kBrushless);
    private final SparkMax motor = new SparkMax(IntakeConstants.kMotorId, MotorType.kBrushless);
    private final MedianFilter currentFilter = new MedianFilter(10); 
    private double filteredCurrent = 0;

    ShuffleboardTab shuffleBoardTab = Shuffleboard.getTab("Intake");
    
    public IntakeSubsystem() {
        //motor.restoreFactoryDefaults();
        motor.setSmartCurrentLimit(30, 40);
        //motor.burnFlash();
        motor.setIdleMode(IdleMode.kBrake);
        setDefaultCommand(stopCommand());
    }

    @Override
    public void periodic() {
        if (LoggingConfig.intakeSubsystemLogging){
            shuffleBoardTab.add("Intake Current", motor.getOutputCurrent());
            shuffleBoardTab.add("Intake Median Current", filteredCurrent);
            shuffleBoardTab.add("Intake Speed", motor.get());
            var currentCommand = this.getCurrentCommand();
            if (currentCommand != null)
            shuffleBoardTab.add("Intake Command", currentCommand.getName());
        }
    }

    // Intake cone, release cube
    public CommandBase intake() {
        return setMotor(IntakeConstants.kIntakeSpeed)
                // .until(this::motorOverCurrent)
                // .andThen(Commands.waitSeconds(IntakeConstants.kIntakeTime))
                // .andThen(stopCommand())
                ;
    }

    private CommandBase setMotor(double speed) {
        return Commands.run(() -> motor.set(speed), this);
    }

    // Release cone, intake cube
    public CommandBase reverseIntake() {
        return setMotor(IntakeConstants.kIntakeSpeed * -1)
                // .until(this::motorOverCurrent)
                // .andThen(Commands.waitSeconds(IntakeConstants.kIntakeTime))
                // .andThen(stopCommand())
                ;
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
