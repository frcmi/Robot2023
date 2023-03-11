package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OperatorConstants;


public class ElevatorSubsystem extends SubsystemBase {
    private final CANSparkMax leftMotor = new CANSparkMax(ElevatorConstants.kLeftMotorId, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax rightMotor = new CANSparkMax(ElevatorConstants.kRightMotorId, CANSparkMaxLowLevel.MotorType.kBrushless);

    private final ProfiledPIDController pidController 
    = new ProfiledPIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD, 
        new TrapezoidProfile.Constraints(ElevatorConstants.kMaxVel, ElevatorConstants.kMaxAccel));

    private final RelativeEncoder encoder = leftMotor.getEncoder();
    public ElevatorSubsystem() {
        leftMotor.restoreFactoryDefaults();
        leftMotor.setSmartCurrentLimit(ElevatorConstants.kCurrentLimit);
        leftMotor.setIdleMode(IdleMode.kBrake);
        leftMotor.burnFlash();
        rightMotor.restoreFactoryDefaults();
        leftMotor.follow(rightMotor, true);
        rightMotor.setSmartCurrentLimit(ElevatorConstants.kCurrentLimit);
        rightMotor.setIdleMode(IdleMode.kBrake);
        rightMotor.burnFlash();

        encoder.setPositionConversionFactor(ElevatorConstants.kElevatorEncoderDistancePerCount);
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
        SmartDashboard.putNumber("Elevator Pos", getPosition());
    }

    public double getPosition() {
        return encoder.getPosition();
    }

    public CommandBase manualMotors(DoubleSupplier input) {
        return run(() -> {
            setMotors(input.getAsDouble());
        });
    }
}
