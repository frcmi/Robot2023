package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ManipulatorConstants;

public class ManipulatorSubsystem extends SubsystemBase {
    private final CANSparkMax elevatorLeftMotor = new CANSparkMax(ManipulatorConstants.kElevatorLeftMotorId, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax elevatorRightMotor = new CANSparkMax(ManipulatorConstants.kElevatorRightMotorId, CANSparkMaxLowLevel.MotorType.kBrushless);

    private final CANSparkMax armLeftMotor = new CANSparkMax(ManipulatorConstants.kArmLeftMotorId, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax armRightMotor = new CANSparkMax(ManipulatorConstants.kArmRightMotorId, CANSparkMaxLowLevel.MotorType.kBrushless);


    public ManipulatorSubsystem() {
        elevatorLeftMotor.restoreFactoryDefaults();
        elevatorRightMotor.restoreFactoryDefaults();
        elevatorRightMotor.follow(elevatorLeftMotor, true);

        armLeftMotor.restoreFactoryDefaults();
        armRightMotor.restoreFactoryDefaults();
        armRightMotor.follow(armLeftMotor, true);
    }

    public void setMotors(double speed) {
        elevatorLeftMotor.set(speed * Constants.OperatorConstants.kElevatorSpeed);
        elevatorRightMotor.set(speed * Constants.OperatorConstants.kElevatorSpeed);

        armLeftMotor.set(speed * Constants.OperatorConstants.kArmSpeed);
        armRightMotor.set(speed * Constants.OperatorConstants.kArmSpeed);
    }
}
