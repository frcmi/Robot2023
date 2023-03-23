// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OperatorConstants;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPRamseteCommand;

import frc.robot.SparkMax;


import frc.robot.Constants.DriveConstants;

import java.util.HashMap;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class DriveSubsystem extends SubsystemBase {
  private final SparkMax frontLeft = new SparkMax(DriveConstants.kFrontLeftMotorId, MotorType.kBrushless);
  private final SparkMax rearLeft = new SparkMax(DriveConstants.kRearLeftMotorId, MotorType.kBrushless);
  private final SparkMax frontRight = new SparkMax(DriveConstants.kFrontRightMotorId, MotorType.kBrushless);
  private final SparkMax rearRight = new SparkMax(DriveConstants.kRearRightMotorId, MotorType.kBrushless);

  private final RelativeEncoder leftEncoder = frontLeft.getEncoder();
  private final RelativeEncoder rightEncoder = frontRight.getEncoder();

  private final MotorControllerGroup leftMotors = new MotorControllerGroup(frontLeft, rearLeft);
  private final MotorControllerGroup rightMotors = new MotorControllerGroup(frontRight, rearRight);
  private final DifferentialDrive diffDrive = new DifferentialDrive(leftMotors, rightMotors);
  private final AHRS navX = new AHRS();

  private Pose2d m_pose = new Pose2d(0, 0, new Rotation2d(0));
  private final Field2d field2d = new Field2d();
  private final DifferentialDriveOdometry m_odometry 
    = new DifferentialDriveOdometry(navX.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition(), m_pose);

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

    leftEncoder.setPositionConversionFactor(DriveConstants.kWheelEncoderDistancePerRotation);
    rightEncoder.setPositionConversionFactor(DriveConstants.kWheelEncoderDistancePerRotation);
    leftEncoder.setVelocityConversionFactor(DriveConstants.kWheelEncoderDistancePerRotation);
    rightEncoder.setVelocityConversionFactor(DriveConstants.kWheelEncoderDistancePerRotation);
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

  public double getPitch() {
    // NavX mounted upside down!
    return navX.getPitch();
  }

  public Rotation2d getHeading() {
    return navX.getRotation2d();
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftMotors.setVoltage(leftVolts);
    rightMotors.setVoltage(rightVolts);
  }

  public void setSpeed(double speed) {
    diffDrive.tankDrive(speed, speed);
  }

  public void resetOdometry(Pose2d pose) {
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
    m_odometry.resetPosition(getHeading(), 0, 0, pose);
  }

  public Pose2d getPose() {
    return m_pose;
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftEncoder.getVelocity(), rightEncoder.getVelocity());
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
    Rotation2d gyroAngle = navX.getRotation2d();

    // Update the pose
    m_pose = m_odometry.update(gyroAngle,
      leftEncoder.getPosition(),
      rightEncoder.getPosition());

    field2d.setRobotPose(m_pose);

    SmartDashboard.putNumber("Gyro Pitch", getPitch());
    SmartDashboard.putNumber("Gyro Heading", getHeading().getDegrees());
    SmartDashboard.putNumber("Drive Left Encoder Pos", leftEncoder.getPosition());
    SmartDashboard.putNumber("Drive Right Encoder Pos", rightEncoder.getPosition());
    SmartDashboard.putNumber("Drive Left Encoder Vel", leftEncoder.getVelocity());
    SmartDashboard.putNumber("Drive Right Encoder Vel", rightEncoder.getVelocity());
    SmartDashboard.putNumber("Front Left Voltage", frontLeft.getBusVoltage());
    SmartDashboard.putNumber("Front Right Voltage", frontRight.getBusVoltage());
    SmartDashboard.putData("Field", field2d);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public CommandBase followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
    return runOnce(() -> {
          // Reset odometry for the first path you run during auto
          if(isFirstPath){
              this.resetOdometry(traj.getInitialPose());
          }

          field2d.getRobotObject().setTrajectory(traj);
        }).andThen(
        new PPRamseteCommand(
            traj, 
            this::getPose, // Pose supplier
            new RamseteController(),
            DriveConstants.feedforward,
            DriveConstants.kinematics, // DifferentialDriveKinematics
            this::getWheelSpeeds, // DifferentialDriveWheelSpeeds supplier
            new PIDController(1, 0, 0), // Left controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            new PIDController(1, 0, 0), // Right controller (usually the same values as left controller)
            this::tankDriveVolts, // Voltage biconsumer
            false, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
            this // Requires this drive subsystem
        ));
  }

  public FollowPathWithEvents getPathWithEvents(PathPlannerTrajectory traj) {
    FollowPathWithEvents pathWithEvents = new FollowPathWithEvents(
      followTrajectoryCommand(traj, true),
      traj.getMarkers(),
      DriveConstants.kEventMap
    );
    return pathWithEvents;
  }

}
