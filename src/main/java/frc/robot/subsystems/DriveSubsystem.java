// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.BalanceCommand;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;
import frc.robot.SparkMax;
import frc.robot.Constants.DriveConstants;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.kauailabs.navx.frc.AHRS;

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
  private final DifferentialDrive m_drive = new DifferentialDrive(leftMotors, rightMotors);
  private final DifferentialDriveKinematics kDriveKinematics =
  new DifferentialDriveKinematics(DriveConstants.kTrackwidthMeters);
  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(DriveConstants.kS, DriveConstants.kV, DriveConstants.kA); 

  public final PhotonvisionSubsystem m_PhotonvisionSubsystem;
  public final RamseteController ramseteController = new RamseteController();
  
  private final AHRS navX = new AHRS();
  private final Gyro m_gyro = navX;

  //Field2d Sim
  private final Field2d m_field = new Field2d();

  //Some smartdashboard variables
  private double y_Displacement = 0.0; 
  private double x_Displacement = 0.0; 
  private final double two = 2.0; 
  private String i_j_Displacement = "";

  private Pose2d pose;

     /* Here we use DifferentialDrivePoseEstimator so that we can fuse odometry readings. The
  numbers used  below are robot specific, and should be tuned. */
  private final DifferentialDrivePoseEstimator m_poseEstimator;

  /** DriveSubsystem Constructer */
  public DriveSubsystem(PhotonvisionSubsystem photonVisionSubsystem) {
    m_PhotonvisionSubsystem = photonVisionSubsystem;
    pose = new Pose2d();
    //frontLeft.restoreFactoryDefaults();
    frontLeft.setIdleMode(IdleMode.kBrake);
    frontLeft.setSmartCurrentLimit(DriveConstants.currentLimit);
    //frontLeft.burnFlash();
    
    //rearLeft.restoreFactoryDefaults();
    rearLeft.setIdleMode(IdleMode.kBrake);
    rearLeft.setSmartCurrentLimit(DriveConstants.currentLimit);
    //rearLeft.burnFlash();

    //frontRight.restoreFactoryDefaults();
    frontRight.setIdleMode(IdleMode.kBrake);
    frontRight.setSmartCurrentLimit(DriveConstants.currentLimit);
    //frontRight.burnFlash();

    //rearRight.restoreFactoryDefaults();
    rearRight.setIdleMode(IdleMode.kBrake);
    rearRight.setSmartCurrentLimit(DriveConstants.currentLimit);
    //rearRight.burnFlash();
    
    rearLeft.follow(frontLeft);
    rearRight.follow(frontRight);
    
    rightMotors.setInverted(true);

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    leftMotors.setInverted(true);
    SmartDashboard.putData("Field", m_field); 
    
    resetEncoders();
    zeroHeading();
    m_poseEstimator = new DifferentialDrivePoseEstimator(
        kDriveKinematics,
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
    // Pose/encoder info
    SmartDashboard.putNumber("Left Encoder", getLeftEncoderDistance());
    SmartDashboard.putNumber("Right Encoder", getRightEncoderDistance());
    SmartDashboard.putNumber("x-displacement", x_Displacement);
    SmartDashboard.putNumber("y-displacement", y_Displacement);
    SmartDashboard.putNumber("orientation", pose.getRotation().getDegrees());
    // Gyro Info
    SmartDashboard.putNumber("Gyro Pitch", getPitch());
    SmartDashboard.putNumber("Gyro Heading", getHeading().getDegrees());
    // DriveTrain info
    SmartDashboard.putNumber("DT Current", frontLeft.getOutputCurrent());
    SmartDashboard.putNumber("DT Speed", frontLeft.get());
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
    return -frontLeft.getEncoder().getPosition() * DriveConstants.kWheelEncoderDistancePerCount;
  }

  public double getRightEncoderDistance() {
    return rearRight.getEncoder().getPosition() * DriveConstants.kWheelEncoderDistancePerCount;
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
    -frontLeft.getEncoder().getVelocity() * DriveConstants.kWheelEncoderDistancePerCount * 10, 
    frontRight.getEncoder().getVelocity() * DriveConstants.kWheelEncoderDistancePerCount * 10);
  }

  public CommandBase resetPoseEstimatorCmd(Pose2d pose) {
    return runOnce(() -> resetPoseEstimator(pose));
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
      // speed = speedFilter.calculate(speed);
      double rotation = rotationSupplier.getAsDouble() * OperatorConstants.kRotationMultiplier;
      m_drive.curvatureDrive(speed, rotation, rotationLock.getAsBoolean());
    });
  }


  public float getPitch() {
    // NavX mounted upside down!
    return navX.getPitch() * -1;
  }

  public CommandBase balanceCommand() {
    return new BalanceCommand(this);
    /*
    return run(() -> {
      double pitchAngleRadians = getPitch() * (Math.PI / 180.0);
      double xAxisRate = Math.sin(pitchAngleRadians);
      tankDriveVolts(xAxisRate * 7, xAxisRate * 7);
      .until(() -> Math.abs(getPitch()) < 3);
    })
    */
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  // Assuming this method is part of a drivetrain subsystem that provides the necessary methods
  public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
    return new SequentialCommandGroup(
        runOnce(() -> {
          // Reset odometry for the first path you run during auto
          // if(isFirstPath){
          //     this.resetOdometry(traj.getInitialPose());
          // }
        }),
        new PPRamseteCommand(
            traj, 
            this::getPose, // Pose supplier
            new RamseteController(),
            this.feedforward,
            this.kDriveKinematics, // DifferentialDriveKinematics
            this::getWheelSpeeds, // DifferentialDriveWheelSpeeds supplier
            new PIDController(0, 0, 0), // Left controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            new PIDController(0, 0, 0), // Right controller (usually the same values as left controller)
            this::tankDriveVolts, // Voltage biconsumer
            true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
            this // Requires this drive subsystem
        )
    );
  }
}
