package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SparkMax;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OperatorConstants;

public class ArmSubsystem extends SubsystemBase {
    private final SparkMax leftMotor = new SparkMax(ArmConstants.kLeftMotorId, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final SparkMax rightMotor = new SparkMax(ArmConstants.kRightMotorId, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final DutyCycleEncoder absoluteEncoder = new DutyCycleEncoder(ArmConstants.kEncoderDIOPort);

    private final ProfiledPIDController pidController 
        = new ProfiledPIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD, 
            new TrapezoidProfile.Constraints(ArmConstants.kMaxVel, ArmConstants.kMaxAccel));
    private final ArmFeedforward feedforward = new ArmFeedforward(ArmConstants.kS, ArmConstants.kG, ArmConstants.kV, ArmConstants.kA);

    double lastSpeed = 0;
    double lastTime = Timer.getFPGATimestamp();

    public ArmSubsystem() {
        leftMotor.setSmartCurrentLimit(ArmConstants.kCurrentLimit);
        leftMotor.setIdleMode(IdleMode.kBrake);
        rightMotor.follow(leftMotor, true);
        rightMotor.setSmartCurrentLimit(ArmConstants.kCurrentLimit);
        rightMotor.setIdleMode(IdleMode.kBrake);

        absoluteEncoder.setDistancePerRotation(1);
        pidController.setGoal(getAngle());
        pidController.setTolerance(Math.toRadians(1.5));
        // would be optimal to use PID as default to hold position
        // but this lets us sway our arm for intaking cones
        setDefaultCommand(stop());
    }

    @Override
    public void periodic() {
        // SmartDashboard.putNumber("Arm Radians", getAngle());
        SmartDashboard.putNumber("Arm Degrees", Math.toDegrees(getAngle()));
        // SmartDashboard.putNumber("Arm Encoder", absoluteEncoder.getAbsolutePosition());
        // SmartDashboard.putData("Arm PID", pidController);
        // SmartDashboard.putNumber("Arm PID Error Deg", Math.toDegrees(pidController.getPositionError()));
    }

    private void setVolts(double volts) {
        double angle = getAngle();
        double kg = feedforward.calculate(angle, 0);
        SmartDashboard.putNumber("Arm Voltage Input", volts);
        // SmartDashboard.putNumber("Arm kg", kg);
        SmartDashboard.putBoolean("Arm Bounds", !(angle > ArmConstants.maxAngle || angle < ArmConstants.minAngle));
        // Stop movement if outside bounds
        if (angle < ArmConstants.minAngle) 
            volts = Math.max(kg, Math.min(2, volts));
        if (angle > ArmConstants.maxAngle)
            volts = Math.max(-2, Math.min(kg, volts));

        // SmartDashboard.putNumber("Arm Voltage Set", volts);
        // SmartDashboard.putNumber("Arm Voltage Left", leftMotor.getAppliedOutput());
        leftMotor.setVoltage(volts);
        rightMotor.setVoltage(volts);
    }

    public double getAngle() {
        return -(absoluteEncoder.getAbsolutePosition() * 2 * Math.PI) + ArmConstants.encoderOffset;
    }

    public void setGoalVolts(double goalAngle) {
        double pidOutput = pidController.calculate(getAngle(), goalAngle);
        State setpoint = pidController.getSetpoint();
        double ffOutpout = feedforward.calculate(setpoint.position, setpoint.velocity);
        SmartDashboard.putNumber("Arm FF Out", ffOutpout);
        SmartDashboard.putNumber("Arm PID Out", pidOutput);
        SmartDashboard.putNumber("Arm Goal Volts", pidOutput + ffOutpout);
        setVolts(pidOutput + ffOutpout);  
    }
      
    public CommandBase stop() {
        return run(() -> {
            double angle = getAngle();
            pidController.reset(angle);
            setVolts(feedforward.calculate(angle, 0));
        });
    }

    public CommandBase moveTo(double angle) {
        CommandBase command = run(() -> setGoalVolts(angle /*+ Math.toRadians(45)*/)).until(pidController::atGoal);
        command.addRequirements(this);
        return command;
    }
}
