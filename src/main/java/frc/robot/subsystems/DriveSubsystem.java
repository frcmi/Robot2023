// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OperatorConstants;

import com.kauailabs.navx.frc.AHRS;
import frc.robot.SparkMax;


import frc.robot.Constants.DriveConstants;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class DriveSubsystem extends SubsystemBase {
  // private final CANSparkMax frontLeft = new CANSparkMax(DriveConstants.kFrontLeftMotorId, MotorType.kBrushless);
  // private final CANSparkMax rearLeft = new CANSparkMax(DriveConstants.kRearLeftMotorId, MotorType.kBrushless);
  // private final CANSparkMax frontRight = new CANSparkMax(DriveConstants.kFrontRightMotorId, MotorType.kBrushless);
  // private final CANSparkMax rearRight = new CANSparkMax(DriveConstants.kRearRightMotorId, MotorType.kBrushless);
  private final SparkMax frontLeft = new SparkMax(DriveConstants.kFrontLeftMotorId, MotorType.kBrushless);
  private final SparkMax rearLeft = new SparkMax(DriveConstants.kRearLeftMotorId, MotorType.kBrushless);
  private final SparkMax frontRight = new SparkMax(DriveConstants.kFrontRightMotorId, MotorType.kBrushless);
  private final SparkMax rearRight = new SparkMax(DriveConstants.kRearRightMotorId, MotorType.kBrushless);


  private final RelativeEncoder leftEncoder = frontLeft.getEncoder();
  private final RelativeEncoder rightEncoder = frontRight.getEncoder();

  private final MotorControllerGroup leftMotors = new MotorControllerGroup(frontLeft, rearLeft);
  private final MotorControllerGroup rightMotors = new MotorControllerGroup(frontRight, rearRight);
  private final DifferentialDrive diffDrive = new DifferentialDrive(leftMotors, rightMotors);
  private final SlewRateLimiter speedFilter = new SlewRateLimiter(OperatorConstants.kSpeedSlewRate);
  private final AHRS navX = new AHRS();

  /** Creates a new ExampleSubsystem. */
  public DriveSubsystem() {
    SmartDashboard.setDefaultNumber("Min turn rate mult", DriveConstants.kMinimumTurnRateMultiplier);
    SmartDashboard.setPersistent("Min turn rate mult");
    //frontLeft.restoreFactoryDefaults();
    frontLeft.setIdleMode(IdleMode.kCoast);
    frontLeft.setSmartCurrentLimit(DriveConstants.currentLimit);
    //frontLeft.burnFlash();
    
    //rearLeft.restoreFactoryDefaults();
    rearLeft.setIdleMode(IdleMode.kCoast);
    rearLeft.setSmartCurrentLimit(DriveConstants.currentLimit);
    //rearLeft.burnFlash();

    //frontRight.restoreFactoryDefaults();
    frontRight.setIdleMode(IdleMode.kCoast);
    frontRight.setSmartCurrentLimit(DriveConstants.currentLimit);
    //frontRight.burnFlash();

    //rearRight.restoreFactoryDefaults();
    rearRight.setIdleMode(IdleMode.kCoast);
    rearRight.setSmartCurrentLimit(DriveConstants.currentLimit);
    //rearRight.burnFlash();
    
    rearLeft.follow(frontLeft);
    rearRight.follow(frontRight);
    
    rightMotors.setInverted(true);

    leftEncoder.setPositionConversionFactor(DriveConstants.kWheelEncoderDistancePerRotation);
    rightEncoder.setPositionConversionFactor(DriveConstants.kWheelEncoderDistancePerRotation);
    leftEncoder.setVelocityConversionFactor(DriveConstants.kWheelEncoderDistancePerRotation);
    rightEncoder.setVelocityConversionFactor(DriveConstants.kWheelEncoderDistancePerRotation);
  }

  /** Gets maximum allowed turn rate (from [0..1]) for a given speed
   * Desmos graph for math used: https://www.desmos.com/calculator/paezdrbpal
   */
  public double getMaxTurnRatePerSpeed(double speed) {
    // Invert the minimum turn rate multiplier so that it's easier to use in the math
    double minimumTurnRateMult = 1 - SmartDashboard.getNumber("Min turn rate mult", DriveConstants.kMinimumTurnRateMultiplier);

    // Calculates speed used in calculation, used for expontential relationship between speed input and max turn rate allowed
    double inputSpeed = Math.abs(Math.pow(speed, DriveConstants.kTurnRateExpontent));

    return (minimumTurnRateMult * -inputSpeed) + 1;

  }
  public CommandBase setSpeed(DoubleSupplier speedSupplier, DoubleSupplier rotationSupplier, BooleanSupplier rotationLock) {
    return run(() -> {
      double speed = speedSupplier.getAsDouble() * OperatorConstants.kSpeedMultiplier;
      // speed = speedFilter.calculate(speed);
      double rotation = rotationSupplier.getAsDouble() * OperatorConstants.kRotationMultiplier;
      double maxAllowedRotation = getMaxTurnRatePerSpeed(speed);
      
      // Clamp the rotation to a maximum of the maximum allowed rotation for the given speed
      double clampedRotation = Math.min(maxAllowedRotation, Math.abs(rotation));

      if (rotation < 0) {
        clampedRotation *= -1;
      }

      diffDrive.arcadeDrive(speed, clampedRotation);
    
    });
  }

  public CommandBase stop() {
    return runOnce(() -> diffDrive.stopMotor());
  }

  public void setBrakes(IdleMode idleMode) {
    frontLeft.setIdleMode(idleMode);
    frontRight.setIdleMode(idleMode);
    rearLeft.setIdleMode(idleMode);
    rearRight.setIdleMode(idleMode);
  }

  public double getRoll() {
    return navX.getRoll();
  }

  public double getHeading() {
    return navX.getAngle();
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftMotors.setVoltage(leftVolts);
    rightMotors.setVoltage(rightVolts);
  }

  public void setSpeed(double speed) {
    diffDrive.tankDrive(speed, speed);
  }

  public CommandBase balanceCommand() {
    CommandBase command = run(() -> {
      double pitchAngleRadians = getRoll() * (Math.PI / 28.0);
      double xAxisRate = Math.sin(pitchAngleRadians) * -1.5;
      SmartDashboard.putNumber("Balance Power", xAxisRate);
      tankDriveVolts(xAxisRate, xAxisRate);
    });
      // .until(() -> Math.abs(getRoll()) < 3);
    command.setName("Balance");
    return runOnce(() -> setBrakes(IdleMode.kBrake)).andThen(command);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Gyro Pitch", navX.getPitch());
    SmartDashboard.putNumber("Gyro Yaw", navX.getYaw());
    SmartDashboard.putNumber("Gyro Roll", navX.getRoll());
    SmartDashboard.putNumber("Drive Left Encoder Pos", leftEncoder.getPosition());
    SmartDashboard.putNumber("Drive Right Encoder Pos", rightEncoder.getPosition());
    SmartDashboard.putNumber("Drive Left Encoder Vel", leftEncoder.getVelocity());
    SmartDashboard.putNumber("Drive Right Encoder Vel", rightEncoder.getVelocity());
    SmartDashboard.putNumber("Front Left Voltage", frontLeft.getAppliedOutput());
    SmartDashboard.putNumber("Rear Left Voltage", rearLeft.getAppliedOutput());
    SmartDashboard.putNumber("Front Right Voltage", frontRight.getAppliedOutput());
    SmartDashboard.putNumber("Rear Right Voltage", rearRight.getAppliedOutput());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
