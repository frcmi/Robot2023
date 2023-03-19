// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OperatorConstants;

import com.ctre.phoenix.CANifier.PinValues;
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


  private final MotorControllerGroup leftMotors = new MotorControllerGroup(frontLeft, rearLeft);
  private final MotorControllerGroup rightMotors = new MotorControllerGroup(frontRight, rearRight);
  private final DifferentialDrive diffDrive = new DifferentialDrive(leftMotors, rightMotors);
  private final SlewRateLimiter speedFilter = new SlewRateLimiter(OperatorConstants.kSpeedSlewRate);
  private final AHRS navX = new AHRS();

  /** Creates a new ExampleSubsystem. */
  public DriveSubsystem() {
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
  }

  public CommandBase setSpeed(DoubleSupplier speedSupplier, DoubleSupplier rotationSupplier, BooleanSupplier rotationLock) {
    return run(() -> {
      double speed = speedSupplier.getAsDouble() * OperatorConstants.kSpeedMultiplier;
      // speed = speedFilter.calculate(speed);
      double rotation = rotationSupplier.getAsDouble() * OperatorConstants.kRotationMultiplier;
      if (rotationLock.getAsBoolean())
        diffDrive.arcadeDrive(0, rotation * 1.5);
      else
        diffDrive.curvatureDrive(speed, rotation, false);
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

  public float getPitch() {
    // NavX mounted upside down!
    return navX.getPitch();
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
      double pitchAngleRadians = getPitch() * (Math.PI / 180.0);
      double xAxisRate = Math.sin(pitchAngleRadians) * 7;
      tankDriveVolts(xAxisRate, xAxisRate);
    })
      .until(() -> Math.abs(getPitch()) < 3);
    command.setName("Balance");
    return command;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Gyro Pitch", getPitch());
    SmartDashboard.putNumber("DT Current", frontLeft.getOutputCurrent());
    SmartDashboard.putNumber("DT Speed", frontLeft.get());
    SmartDashboard.putNumber("Front Left Voltage", frontLeft.getBusVoltage());
    SmartDashboard.putNumber("Rear Left Voltage", rearLeft.getBusVoltage());
    SmartDashboard.putNumber("Front Right Voltage", frontRight.getBusVoltage());
    SmartDashboard.putNumber("Rear Right Voltage", rearRight.getBusVoltage());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
