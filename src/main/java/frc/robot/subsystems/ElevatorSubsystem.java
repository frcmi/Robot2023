package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SparkMax;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OperatorConstants;


public class ElevatorSubsystem extends SubsystemBase {
    //private final CANSparkMax leftMotor = new CANSparkMax(ElevatorConstants.kLeftMotorId, CANSparkMaxLowLevel.MotorType.kBrushless);
    //private final CANSparkMax rightMotor = new CANSparkMax(ElevatorConstants.kRightMotorId, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final SparkMax leftMotor = new SparkMax(ElevatorConstants.kLeftMotorId, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final SparkMax rightMotor = new SparkMax(ElevatorConstants.kRightMotorId, CANSparkMaxLowLevel.MotorType.kBrushless);


    private final ProfiledPIDController pidController 
        = new ProfiledPIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD, 
            new TrapezoidProfile.Constraints(ElevatorConstants.kMaxVel, ElevatorConstants.kMaxAccel));
    private final RelativeEncoder encoder = leftMotor.getEncoder();

    public ElevatorSubsystem() {
        //leftMotor.restoreFactoryDefaults();
        leftMotor.setSmartCurrentLimit(ElevatorConstants.kCurrentLimit);
        leftMotor.setIdleMode(IdleMode.kCoast);
        //leftMotor.burnFlash();
        //rightMotor.restoreFactoryDefaults();
        rightMotor.setInverted(true);
        leftMotor.follow(rightMotor, true);
        rightMotor.setSmartCurrentLimit(ElevatorConstants.kCurrentLimit);
        rightMotor.setIdleMode(IdleMode.kCoast);
        //rightMotor.burnFlash();

        encoder.setPositionConversionFactor(ElevatorConstants.kElevatorEncoderDistancePerCount);
        pidController.setTolerance(0.5);

    }

    @Override
    public void periodic() {
        // SmartDashboard.putData("Elevator PID", pidController);
        // SmartDashboard.putNumber("Elevator Speed", leftMotor.get());
        SmartDashboard.putNumber("Elevator Pos", getPosition());
        setDefaultCommand(run(this::stop));
        Command command = getCurrentCommand();
        if (command != null)
            SmartDashboard.putString("Elevator Command", command.getName());
        else
            SmartDashboard.putString("Elevator Command", "null");
    }

    public double getPosition() {
        return encoder.getPosition();
    }

    private void setVolts(double volts) {
        double position = getPosition();
        boolean outOfBounds = position < ElevatorConstants.minPos || position > ElevatorConstants.maxPos;
        // Stop movement if outside bounds
        if (position < ElevatorConstants.minPos) 
            volts = Math.max(0, Math.min(2, volts));
        if (position > ElevatorConstants.maxPos)
            volts = Math.max(-2, Math.min(0, volts));
        SmartDashboard.putBoolean("Elevator Bounds", !outOfBounds);
        SmartDashboard.putNumber("Elevator Out Volts", volts);
        leftMotor.setVoltage(volts);
        rightMotor.setVoltage(volts);
    }

    public CommandBase stop() {
        CommandBase command = run(() -> setVolts(ElevatorConstants.kG));
        command.setName("ElevatorStop");
        return command;
    }

    public void setGoalVolts(double goalPosition) {
        double pidOutput = pidController.calculate(getPosition(), goalPosition);
        pidOutput *= 25;
        setVolts(pidOutput + ElevatorConstants.kG);
    }

    public CommandBase moveTo(double position) {
        return run(() -> setGoalVolts(position)).until(pidController::atGoal);
    }

    public CommandBase raise() {
        CommandBase command = moveTo(0.4);
        command.setName("RaiseElevator");
        return command;
    }

    public CommandBase lower() {
        CommandBase command = moveTo(0);
        command.setName("LowerElevator");
        return command;
    }
}
