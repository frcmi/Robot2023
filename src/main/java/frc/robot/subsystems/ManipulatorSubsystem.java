package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ManipulatorConstants;

public class ManipulatorSubsystem extends SubsystemBase {
    private final CANSparkMax elevatorLeftMotor = new CANSparkMax(ManipulatorConstants.kElevatorLeftMotorId, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax elevatorRightMotor = new CANSparkMax(ManipulatorConstants.kElevatorRightMotorId, CANSparkMaxLowLevel.MotorType.kBrushless);

    private final CANSparkMax armLeftMotor = new CANSparkMax(ManipulatorConstants.kArmLeftMotorId, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax armRightMotor = new CANSparkMax(ManipulatorConstants.kArmRightMotorId, CANSparkMaxLowLevel.MotorType.kBrushless);


    public ManipulatorSubsystem() {
        elevatorLeftMotor.restoreFactoryDefaults();
        elevatorLeftMotor.setSmartCurrentLimit(ManipulatorConstants.kElevatorCurrentLimit);
        elevatorLeftMotor.burnFlash();
        elevatorRightMotor.restoreFactoryDefaults();
        elevatorRightMotor.follow(elevatorLeftMotor, true);
        elevatorRightMotor.setSmartCurrentLimit(ManipulatorConstants.kElevatorCurrentLimit);
        elevatorRightMotor.burnFlash();

        armLeftMotor.restoreFactoryDefaults();
        armLeftMotor.setSmartCurrentLimit(ManipulatorConstants.kArmCurrentLimit);
        armLeftMotor.burnFlash();
        armRightMotor.restoreFactoryDefaults();
        armRightMotor.follow(armLeftMotor, true);
        armRightMotor.setSmartCurrentLimit(ManipulatorConstants.kArmCurrentLimit);
        armRightMotor.burnFlash();
    }

    public void setMotors(double speed) {
        setElevatorMotors(speed);
        setArmMotors(speed);
    }

    private void setArmMotors(double speed) {
        speed *= OperatorConstants.kArmSpeed;

        armLeftMotor.set(speed);
        armRightMotor.set(speed);
    }

    private void setElevatorMotors(double speed) {
        speed *= OperatorConstants.kElevatorSpeed;

        elevatorLeftMotor.set(speed);
        elevatorRightMotor.set(speed);
    }
}
