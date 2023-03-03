package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OperatorConstants;

public class ArmSubsystem extends SubsystemBase {
    private final CANSparkMax leftMotor = new CANSparkMax(ArmConstants.kLeftMotorId, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax rightMotor = new CANSparkMax(ArmConstants.kRightMotorId, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final AbsoluteEncoder absoluteEncoder = leftMotor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);

    private final ProfiledPIDController pidController 
        = new ProfiledPIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD, 
            new TrapezoidProfile.Constraints(ArmConstants.kMaxVel, ArmConstants.kMaxAccel));
//    private final ArmFeedforward feedforward = new ArmFeedforward(ArmConstants.kS, ArmConstants.kG, ArmConstants.kV, ArmConstants.kA);
    public ArmSubsystem() {
        leftMotor.restoreFactoryDefaults();
        leftMotor.setSmartCurrentLimit(ArmConstants.kCurrentLimit);
        leftMotor.burnFlash();
        rightMotor.restoreFactoryDefaults();
        rightMotor.follow(leftMotor, true);
        rightMotor.setSmartCurrentLimit(ArmConstants.kCurrentLimit);
        rightMotor.burnFlash();

        pidController.setTolerance(1);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Angle", getAngle());
        SmartDashboard.putData("Arm PID", pidController);
        SmartDashboard.putNumber("Arm Speed", leftMotor.get());
    }

    private void setMotors(double speed) {
        speed *= OperatorConstants.kArmSpeed;
        leftMotor.set(speed);
        rightMotor.set(speed);
    }

    public double getAngle() {
        return absoluteEncoder.getPosition();
    }

    private void setPID() {
        double pidOutput = pidController.calculate(getAngle());
        leftMotor.set(pidOutput);
        rightMotor.set(pidOutput);
    }

    public CommandBase pidArm() {
        return run(this::setPID);
    }

    public CommandBase manualMove(DoubleSupplier inputSupplier) {
        return run(() -> {
            double input = inputSupplier.getAsDouble();
            if (Math.abs(input) < OperatorConstants.ArmDeadzone) {
                pidArm();
                return;
            }
            setMotors(input);
        });
    }

    public CommandBase setTarget(double angle) {
        return runOnce(() -> pidController.setGoal(angle));
    }

    public CommandBase moveArmTo(double angle) {
        return setTarget(angle).andThen(pidArm()).until(pidController::atSetpoint);
    }

    public CommandBase moveArmToRelative(double angleOffset) {
        return moveArmTo(absoluteEncoder.getPosition() + angleOffset);
    }
}
