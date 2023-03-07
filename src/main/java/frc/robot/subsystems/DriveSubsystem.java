// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.BalanceCommand;

import com.kauailabs.navx.frc.AHRS;

import frc.robot.Constants.DriveConstants;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class DriveSubsystem extends SubsystemBase {
  private final CANSparkMax frontLeft = new CANSparkMax(DriveConstants.kFrontLeftMotorId, MotorType.kBrushless);
  private final CANSparkMax rearLeft = new CANSparkMax(DriveConstants.kRearLeftMotorId, MotorType.kBrushless);
  private final CANSparkMax frontRight = new CANSparkMax(DriveConstants.kFrontRightMotorId, MotorType.kBrushless);
  private final CANSparkMax rearRight = new CANSparkMax(DriveConstants.kRearRightMotorId, MotorType.kBrushless);

  private final MotorControllerGroup leftMotors = new MotorControllerGroup(frontLeft, rearLeft);
  private final MotorControllerGroup rightMotors = new MotorControllerGroup(frontRight, rearRight);
  private final DifferentialDrive diffDrive = new DifferentialDrive(leftMotors, rightMotors);
  private final SlewRateLimiter speedFilter = new SlewRateLimiter(OperatorConstants.kSpeedSlewRate);
  private final AHRS navX = new AHRS();
  private final DifferentialDriveOdometry m_odometry;


  /** Creates a new ExampleSubsystem. */
  public DriveSubsystem() {
    frontLeft.restoreFactoryDefaults();
    rearLeft.restoreFactoryDefaults();
    frontRight.restoreFactoryDefaults();
    rearRight.restoreFactoryDefaults();
    
    rearLeft.follow(frontLeft);
    rearRight.follow(frontRight);
    
    rightMotors.setInverted(true);

    m_odometry = new DifferentialDriveOdometry(getHeading(), 0, 0, new Pose2d()); 
  }

  public CommandBase setSpeed(DoubleSupplier speedSupplier, DoubleSupplier rotationSupplier, BooleanSupplier rotationLock) {
    return run(() -> {
      double speed = speedSupplier.getAsDouble() * OperatorConstants.kSpeedMultiplier;
      speed = speedFilter.calculate(speed);
      double rotation = rotationSupplier.getAsDouble() * OperatorConstants.kRotationMultiplier;
      diffDrive.curvatureDrive(speed, rotation, rotationLock.getAsBoolean());
    });
  }

  public CommandBase stop() {
    return runOnce(() -> diffDrive.stopMotor());
  }

  public float getPitch() {
    // NavX mounted upside down!
    return navX.getPitch() * -1;
  }

  public Rotation2d getHeading() {
    return navX.getRotation2d() /* .plus(Rotation2d.fromDegrees(180))*/;
  }

  public void zeroHeading() {
    navX.reset();
  }

  public double getTurnRate() {
    return navX.getRate();
  }

  public double getLeftEncoderDistance() {
    return -frontLeft.getEncoder().getPosition() * frontLeft.getEncoder().getCountsPerRevolution();
  }

  public double getRightEncoderDistance() {
    return rearRight.getEncoder().getPosition() * rearRight.getEncoder().getCountsPerRevolution();
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public CommandBase resetOdometryCmd(Pose2d pose) {
    return Commands.runOnce(() -> resetOdometry(pose), this);
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(getHeading(), 0, 0, pose);
  }

  public void resetEncoders() {
    frontLeft.getEncoder().setPosition(0.0);
    rearLeft.getEncoder().setPosition(0.0);
    frontRight.getEncoder().setPosition(0.0);
    rearRight.getEncoder().setPosition(0.0);
   }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() { //in m/s
    return new DifferentialDriveWheelSpeeds(
    -frontLeft.getEncoder().getVelocity() * DriveConstants.kWheelEncoderDistancePerCount * 10, 
    frontRight.getEncoder().getVelocity() * DriveConstants.kWheelEncoderDistancePerCount * 10);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftMotors.setVoltage(leftVolts);
    rightMotors.setVoltage(rightVolts);
  }

  public CommandBase balanceCommand() {
    return new BalanceCommand(this);
    /*
    return run(() -> {
      double pitchAngleRadians = getPitch() * (Math.PI / 180.0);
      double xAxisRate = Math.sin(pitchAngleRadians);
      tankDriveVolts(xAxisRate * 7, xAxisRate * 7);
    })
      .until(() -> Math.abs(getPitch()) < 3);
    */
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Gyro Pitch", getPitch());
    //DT stands for DriveTrain
    SmartDashboard.putNumber("DT Current", frontLeft.getOutputCurrent());
    SmartDashboard.putNumber("DT Speed", frontLeft.get());
    SmartDashboard.putBoolean("Is Balancing?", balanceCommand().isScheduled());
    m_odometry.update(
        getHeading(), 
        getLeftEncoderDistance(), 
        getRightEncoderDistance());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
