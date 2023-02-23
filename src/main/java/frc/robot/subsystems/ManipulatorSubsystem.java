package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ManipulatorConstants;

public class ManipulatorSubsystem extends SubsystemBase {
    private final WPI_TalonFX elevatorLeftMotor = new WPI_TalonFX(ManipulatorConstants.kElevatorLeftMotorId);
    private final WPI_TalonFX elevatorRightMotor = new WPI_TalonFX(ManipulatorConstants.kElevatorRightMotorId);


    public ManipulatorSubsystem() {
        elevatorLeftMotor.configFactoryDefault();
        elevatorRightMotor.configFactoryDefault();
        
        // Set one to inverted, not sure which one right now
        // leftMotor.setInverted(true);
        elevatorRightMotor.setInverted(true);
    }

    public void setMotors(double speed) {
        elevatorLeftMotor.set(speed);
        elevatorRightMotor.set(speed);
    }
}
