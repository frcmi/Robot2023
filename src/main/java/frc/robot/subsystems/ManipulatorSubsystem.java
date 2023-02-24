package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OperatorConstants;

public class ManipulatorSubsystem extends SubsystemBase {
    private final ArmSubsystem m_armSubsystem;
    private final ElevatorSubsystem m_elevatorSubsystem;

    public ManipulatorSubsystem(ArmSubsystem armSubsystem, ElevatorSubsystem elevatorSubsystem) {
        m_armSubsystem = armSubsystem;
        m_elevatorSubsystem = elevatorSubsystem;
    }

    public void setMotors(double speed) {
        m_armSubsystem.setMotors(speed);
        m_elevatorSubsystem.setMotors(speed);
    }
}
