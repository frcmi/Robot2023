package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OperatorConstants;

public class ArmSubsystem {
    private final CANSparkMax leftMotor = new CANSparkMax(ArmConstants.kLeftMotorId, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax rightMotor = new CANSparkMax(ArmConstants.kRightMotorId, CANSparkMaxLowLevel.MotorType.kBrushless);


    public ArmSubsystem() {
        leftMotor.restoreFactoryDefaults();
        leftMotor.setSmartCurrentLimit(ArmConstants.kCurrentLimit);
        leftMotor.burnFlash();
        rightMotor.restoreFactoryDefaults();
        rightMotor.follow(leftMotor, true);
        rightMotor.setSmartCurrentLimit(ArmConstants.kCurrentLimit);
        rightMotor.burnFlash();
    }

    public void setMotors(double speed) {
        speed *= OperatorConstants.kArmSpeed;
        leftMotor.set(speed);
        rightMotor.set(speed);
    }
}
