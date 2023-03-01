package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OperatorConstants;


public class ElevatorSubsystem extends SubsystemBase {
    private final CANSparkMax leftMotor = new CANSparkMax(ElevatorConstants.kLeftMotorId, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax rightMotor = new CANSparkMax(ElevatorConstants.kRightMotorId, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final PIDController pidController = new PIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);

    public ElevatorSubsystem() {
        leftMotor.restoreFactoryDefaults();
        leftMotor.setSmartCurrentLimit(ElevatorConstants.kCurrentLimit);
        leftMotor.burnFlash();
        rightMotor.restoreFactoryDefaults();
        rightMotor.follow(leftMotor, true);
        rightMotor.setSmartCurrentLimit(ElevatorConstants.kCurrentLimit);
        rightMotor.burnFlash();
    }

    private void setMotors(double speed) {
        speed *= OperatorConstants.kElevatorSpeed;
        leftMotor.set(speed);
        rightMotor.set(speed);
    }

    @Override
    public void periodic() {
        SmartDashboard.putData("Elevator PID", pidController);
        SmartDashboard.putNumber("Elevator Speed", leftMotor.get());
    }

    public CommandBase manualMotors(DoubleSupplier input) {
        return run(() -> {
            setMotors(input.getAsDouble());
        });
    }
}
