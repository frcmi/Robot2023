// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OperatorConstants;

import static frc.robot.Constants.DriveConstants;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.kauailabs.navx.frc.AHRS;

public class DriveSubsystem extends SubsystemBase {
  private final CANSparkMax frontLeft = new CANSparkMax(DriveConstants.kFrontLeftMotorId, MotorType.kBrushless);
  private final CANSparkMax rearLeft = new CANSparkMax(DriveConstants.kRearLeftMotorId, MotorType.kBrushless);
  private final CANSparkMax frontRight = new CANSparkMax(DriveConstants.kFrontRightMotorId, MotorType.kBrushless);
  private final CANSparkMax rearRight = new CANSparkMax(DriveConstants.kRearRightMotorId, MotorType.kBrushless);

  private final MotorControllerGroup leftMotors = new MotorControllerGroup(frontLeft, rearLeft);
  private final MotorControllerGroup rightMotors = new MotorControllerGroup(frontRight, rearRight);
  private final DifferentialDrive m_drive = new DifferentialDrive(leftMotors, rightMotors);

  public final PhotonvisionSubsystem m_PhotonvisionSubsystem;
  
  // The gyro sensor (Pigeon 2)
  private final AHRS navX = new AHRS();
  private final Gyro m_gyro = navX;

  //Field2d Sim
  private final Field2d m_field = new Field2d();

  //Some smartdashboard variables
  private double y_Displacement = 0.0; 
  private double x_Displacement = 0.0; 
  private final double two = 2.0; 
  private String i_j_Displacement = "";

  Pose2d pose;

     /* Here we use DifferentialDrivePoseEstimator so that we can fuse odometry readings. The
  numbers used  below are robot specific, and should be tuned. */
  private final DifferentialDrivePoseEstimator m_poseEstimator;

  /** DriveSubsystem Constructer */
  public DriveSubsystem(PhotonvisionSubsystem photonVisionSubsystem) {
    m_PhotonvisionSubsystem = photonVisionSubsystem;
    pose = new Pose2d();

    frontLeft.restoreFactoryDefaults();
    rearLeft.restoreFactoryDefaults();
    frontRight.restoreFactoryDefaults();
    rearRight.restoreFactoryDefaults();

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    leftMotors.setInverted(true);
    SmartDashboard.putData("Field", m_field); 
    
    resetEncoders();
    zeroHeading();
    m_poseEstimator = new DifferentialDrivePoseEstimator(
        DriveConstants.kDriveKinematics,
        m_gyro.getRotation2d(),
        getLeftEncoderDistance(),
        getRightEncoderDistance(),
        pose);
  }

  @Override
  public void periodic() {
    updateOdometry();
    m_field.setRobotPose(pose);

    x_Displacement = pose.getX(); 
    y_Displacement = pose.getY(); 
    i_j_Displacement = x_Displacement + "i + " + y_Displacement + "j"; 
   
    SmartDashboard.putString("Polar Displacement", "[" + Math.sqrt(Math.pow(x_Displacement, 
    2) + Math.pow(y_Displacement, two)) + " " 
    + Math.atan(y_Displacement/x_Displacement) + "]");

    SmartDashboard.putString("Rectangular Displacement", i_j_Displacement);
    
    SmartDashboard.putNumber("Gyro", getHeading().getDegrees());
    SmartDashboard.putNumber("Left Encoder", getLeftEncoderDistance());
    SmartDashboard.putNumber("Right Encoder", getRightEncoderDistance());
    SmartDashboard.putNumber("x-displacement", x_Displacement);
    SmartDashboard.putNumber("y-displacement", y_Displacement);
    SmartDashboard.putNumber("orientation", pose.getRotation().getDegrees());
  }

  public Pose2d getInitialPose() {
    PhotonTrackedTarget target = m_PhotonvisionSubsystem.camera.getLatestResult().getBestTarget();
    Transform3d transform = target.getBestCameraToTarget();
    return new Pose2d(transform.getX(), transform.getY(), transform.getRotation().toRotation2d());
  }

  /** Updates the field-relative position. */
  public void updateOdometry() {
    Optional<EstimatedRobotPose> estimatedPose = m_PhotonvisionSubsystem.getEstimatedGlobalPose(pose);
    if (estimatedPose.isPresent())
      pose = estimatedPose.get().estimatedPose.toPose2d();

    m_poseEstimator.update(m_gyro.getRotation2d(), getLeftEncoderDistance(), getRightEncoderDistance());
    Optional<EstimatedRobotPose> result = m_PhotonvisionSubsystem.getEstimatedGlobalPose(m_poseEstimator.getEstimatedPosition());    
    
    if (result.isPresent()) {
      EstimatedRobotPose camPose = result.get();
      Pose2d camPose2d = camPose.estimatedPose.toPose2d();
      m_poseEstimator.addVisionMeasurement(camPose2d, camPose.timestampSeconds);
      m_field.getObject("Cam Est Pos").setPose(camPose2d);
    }

    m_field.getObject("Actual Pos").setPose(getPose());
    m_field.setRobotPose(m_poseEstimator.getEstimatedPosition());
  }
  public double getLeftEncoderDistance() {
    return -frontLeft.getEncoder().getPosition() * DriveConstants.kEncoderDistancePerPulse;
  }

  public double getRightEncoderDistance() {
    return rearRight.getEncoder().getPosition() * DriveConstants.kEncoderDistancePerPulse;
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() { 
    return m_poseEstimator.getEstimatedPosition();
   }


  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() { //in m/s
    return new DifferentialDriveWheelSpeeds(
    -frontLeft.getEncoder().getVelocity() * DriveConstants.kEncoderDistancePerPulse * 10, 
    frontRight.getEncoder().getVelocity() * DriveConstants.kEncoderDistancePerPulse * 10);
  }

  public CommandBase resetPoseEstimatorCmd(Pose2d pose) {
    return Commands.runOnce(() -> resetPoseEstimator(pose), this);
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetPoseEstimator(Pose2d pose) {
    resetEncoders();
    m_poseEstimator.resetPosition(getHeading(), 0, 0, pose);
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }

  public CommandBase stop() {
    return Commands.runOnce(m_drive::stopMotor, this);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftMotors.setVoltage(leftVolts);
    rightMotors.setVoltage(rightVolts);
    m_drive.feed();
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
   frontLeft.getEncoder().setPosition(0.0);
   rearLeft.getEncoder().setPosition(0.0);
   frontRight.getEncoder().setPosition(0.0);
   rearRight.getEncoder().setPosition(0.0);
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (getLeftEncoderDistance() + getRightEncoderDistance()) / 2.0;
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public Rotation2d getHeading() {
    return m_gyro.getRotation2d() /* .plus(Rotation2d.fromDegrees(180))*/;
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate();
  }



  public CommandBase setSpeed(DoubleSupplier speedSupplier, DoubleSupplier rotationSupplier, BooleanSupplier rotationLock) {
    return run(() -> {
      double speed = speedSupplier.getAsDouble() * OperatorConstants.kSpeedMultiplier;
      double rotation = rotationSupplier.getAsDouble() * OperatorConstants.kRotationMultiplier;
      m_drive.curvatureDrive(speed, rotation, rotationLock.getAsBoolean());
    });
  }


  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }


  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
