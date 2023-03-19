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
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SparkMax;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OperatorConstants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;


public class ElevatorSubsystem extends SubsystemBase implements Loggable {
    //private final CANSparkMax leftMotor = new CANSparkMax(ElevatorConstants.kLeftMotorId, CANSparkMaxLowLevel.MotorType.kBrushless);
    //private final CANSparkMax rightMotor = new CANSparkMax(ElevatorConstants.kRightMotorId, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final SparkMax leftMotor = new SparkMax(ElevatorConstants.kLeftMotorId, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final SparkMax rightMotor = new SparkMax(ElevatorConstants.kRightMotorId, CANSparkMaxLowLevel.MotorType.kBrushless);


    @Log
    private final ProfiledPIDController pidController 
        = new ProfiledPIDController(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD, 
            new TrapezoidProfile.Constraints(ElevatorConstants.kMaxVel, ElevatorConstants.kMaxAccel));

    private final ElevatorFeedforward feedforward 
        = new ElevatorFeedforward(ElevatorConstants.kS, ElevatorConstants.kG, ElevatorConstants.kV, ElevatorConstants.kA);
    @Log.Encoder(name = "Elevator Encoder")
    private final RelativeEncoder encoder = leftMotor.getEncoder();

    public ElevatorSubsystem() {
        //leftMotor.restoreFactoryDefaults();
        leftMotor.setSmartCurrentLimit(ElevatorConstants.kCurrentLimit);
        leftMotor.setIdleMode(IdleMode.kBrake);
        //leftMotor.burnFlash();
        //rightMotor.restoreFactoryDefaults();
        leftMotor.follow(rightMotor, true);
        rightMotor.setSmartCurrentLimit(ElevatorConstants.kCurrentLimit);
        rightMotor.setIdleMode(IdleMode.kBrake);
        //rightMotor.burnFlash();

        encoder.setPositionConversionFactor(ElevatorConstants.kElevatorEncoderDistancePerCount);
    }

    private void setMotors(double speed) {
        speed *= OperatorConstants.kElevatorSpeed;
        leftMotor.set(speed);
        rightMotor.set(speed);
    }

    @Log
    public double getSpeed() {
        return leftMotor.get();
    }

    @Log
    public double getPosition() {
        return encoder.getPosition();
    }

    private void setVolts(double volts) {
        double position = getPosition();
        // Stop movement if outside bounds
        if (position < ElevatorConstants.minPos || position > ElevatorConstants.maxPos)
            volts = 0;
        leftMotor.setVoltage(volts);
        rightMotor.setVoltage(volts);
    }

    public void stop() {
        setVolts(feedforward.calculate(0));
    }

    public void setGoalVolts(double goalPosition) {
        double pidOutput = pidController.calculate(getPosition(), goalPosition);
        State setpoint = pidController.getSetpoint();
        double ffOutpout = feedforward.calculate(setpoint.velocity);
        setVolts(pidOutput + ffOutpout);
    }

    public CommandBase manualMotors(DoubleSupplier input) {
        return run(() -> {
            setMotors(input.getAsDouble());
        });
    }

    public CommandBase moveTo(double position) {
        return run(() -> setGoalVolts(position)).until(pidController::atGoal).andThen(this::stop);
    }

    public CommandBase moveToRelative(double positionOffset) {
        return moveTo(getPosition() + positionOffset);
    }
}
