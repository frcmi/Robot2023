package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
    private final WPI_TalonFX leftMotor = new WPI_TalonFX(ElevatorConstants.kElevatorLeftMotorId);
    private final WPI_TalonFX rightMotor = new WPI_TalonFX(ElevatorConstants.kElevatorRightMotorId);

    public ElevatorSubsystem() {
        leftMotor.configFactoryDefault();
        rightMotor.configFactoryDefault();
        
        // Set one to inverted, not sure which one right now
        // leftMotor.setInverted(true);
        rightMotor.setInverted(true);
    }

    public void setMotors(double speed) {
        leftMotor.set(speed);
        rightMotor.set(speed);
    }
}
