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
    private final SlewRateLimiter speedFilter = new SlewRateLimiter(OperatorConstants.kSpeedSlewRate);
    private final ArmFeedforward feedforward = new ArmFeedforward(ArmConstants.kS, ArmConstants.kG, ArmConstants.kV, ArmConstants.kA);

    double lastSpeed = 0;
    double lastTime = Timer.getFPGATimestamp();

    public ArmSubsystem() {
        leftMotor.setSmartCurrentLimit(ArmConstants.kCurrentLimit);
        leftMotor.setIdleMode(IdleMode.kCoast);
        rightMotor.follow(leftMotor, true);
        rightMotor.setSmartCurrentLimit(ArmConstants.kCurrentLimit);
        rightMotor.setIdleMode(IdleMode.kCoast);

        absoluteEncoder.setDistancePerRotation(1);
        pidController.setGoal(getAngle());
        pidController.setTolerance(Math.toRadians(1));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Radians", getAngle());
        SmartDashboard.putNumber("Arm Degrees", Math.toDegrees(getAngle()));
        SmartDashboard.putNumber("Arm Encoder", absoluteEncoder.get());
        SmartDashboard.putData("Arm PID", pidController);
        SmartDashboard.putNumber("Arm PID Error", Math.toDegrees(pidController.getPositionError()));
        SmartDashboard.putNumber("Arm Current", leftMotor.getOutputCurrent());
    }

    private void setSpeed(double speed) {
        speed *= OperatorConstants.kArmSpeed;

        double angle = getAngle();
        // Stop movement if outside bounds
        if (angle < ArmConstants.minAngle || angle > ArmConstants.maxAngle)
            speed = 0;
        leftMotor.set(speed);
        rightMotor.set(speed);
    }

    private void setVolts(double volts) {
        double angle = getAngle();
        // Stop movement if outside bounds
        if (angle < ArmConstants.minAngle || angle > ArmConstants.maxAngle)
            volts = 0;
        leftMotor.setVoltage(volts);
        rightMotor.setVoltage(volts);
    }

    public CommandBase manualMotors(DoubleSupplier input) {
        return run(() -> {
            setSpeed(input.getAsDouble());
        });
    }

    public double getAngle() {
        return -(absoluteEncoder.getAbsolutePosition() * 2 * Math.PI) + ArmConstants.encoderOffset;
    }

    public CommandBase manualMove(DoubleSupplier inputSupplier) {
        return run(() -> {
            double input = inputSupplier.getAsDouble();
            input = speedFilter.calculate(input);
            if (Math.abs(input) < OperatorConstants.kArmDeadzone) {
                // pidMotors();
                return;
            }
            setSpeed(input);
            pidController.setGoal(getAngle());
        });
    }

    public void setGoalVolts(double goalAngle) {
        double pidOutput = pidController.calculate(getAngle(), goalAngle);
        State setpoint = pidController.getSetpoint();
        double ffOutpout = feedforward.calculate(setpoint.position, setpoint.velocity);
        setVolts(pidOutput + ffOutpout);  
    }
      
    public CommandBase stop() {
        return run(() -> setVolts(feedforward.kg));
    }

    public CommandBase setTarget(double angle) {
        return runOnce(() -> pidController.setGoal(angle));
    }

    public CommandBase moveTo(double angle) {
        return run(() -> setGoalVolts(angle)).until(pidController::atGoal).andThen(stop());
    }

    public CommandBase moveToRelative(double angleOffset) {
        return moveTo(getAngle() + angleOffset);
    }
}
