// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.hal.EncoderJNI;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDeviceJNI;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.EncoderDataJNI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.RobotController;

import static frc.robot.Constants.DriveConstants;
import static frc.robot.Constants.AutoConstants;
import static frc.robot.Constants.OperatorConstants;

public class DriveSubsystem extends SubsystemBase {
  private final CANSparkMax frontLeft = new CANSparkMax(DriveConstants.kFrontLeftMotorId, MotorType.kBrushless);
  private final CANSparkMax rearLeft = new CANSparkMax(DriveConstants.kRearLeftMotorId, MotorType.kBrushless);
  private final CANSparkMax frontRight = new CANSparkMax(DriveConstants.kFrontRightMotorId, MotorType.kBrushless);
  private final CANSparkMax rearRight = new CANSparkMax(DriveConstants.kRearRightMotorId, MotorType.kBrushless);
  private final RelativeEncoder leftEncoder = frontLeft.getEncoder();
  private final RelativeEncoder rightEncoder = frontRight.getEncoder();
  private SimDeviceSim leftSparkSim = new SimDeviceSim("SPARK MAX [1]");
  private SimDeviceSim rightSparkSim = new SimDeviceSim("SPARK MAX [2]");
  private final AHRS ahrs = new AHRS(Port.kMXP);
  private final Field2d field = new Field2d();
  private final MotorControllerGroup leftMotors = new MotorControllerGroup(frontLeft, rearLeft);
  private final MotorControllerGroup rightMotors = new MotorControllerGroup(frontRight, rearRight);
  private final DifferentialDrive diffDrive = new DifferentialDrive(leftMotors, rightMotors);
  public DifferentialDrivetrainSim diffSim = new DifferentialDrivetrainSim(DCMotor.getNEO(1), DriveConstants.kGearRatio, DriveConstants.kMomentOfInertia, DriveConstants.kMass, DriveConstants.kWheelDiameterMeters/2, DriveConstants.kTrackwidthMeters, null);
  public DifferentialDriveOdometry odometry;
  private double gyroSimAngle = 0.0;

  /** Creates a new ExampleSubsystem. */
  public DriveSubsystem(Pose2d startingPose) {
    frontLeft.restoreFactoryDefaults();
    rearLeft.restoreFactoryDefaults();
    frontRight.restoreFactoryDefaults();
    rearRight.restoreFactoryDefaults();
    rearLeft.follow(frontLeft);
    rearRight.follow(frontRight);
    leftMotors.setInverted(true);
    odometry = new DifferentialDriveOdometry(ahrs.getRotation2d(),leftEncoder.getPosition(), rightEncoder.getPosition(), startingPose);
    SmartDashboard.putData("Field", field);
  }

  public DriveSubsystem() {
    frontLeft.restoreFactoryDefaults();
    rearLeft.restoreFactoryDefaults();
    frontRight.restoreFactoryDefaults();
    rearRight.restoreFactoryDefaults();
    rearLeft.follow(frontLeft);
    rearRight.follow(frontRight);
    leftMotors.setInverted(true);
    odometry = new DifferentialDriveOdometry(ahrs.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition(), new Pose2d(5.0, 5.5, new Rotation2d()));
    SmartDashboard.putData("Field", field);
  }

  public DifferentialDriveOdometry getOdometry() {
    return odometry;
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  public CommandBase setSpeed(double speed, double rotation) {
    frontLeft.set(-speed / RobotController.getInputVoltage());
    frontRight.set(speed / RobotController.getInputVoltage());

    return runOnce(() -> diffDrive.arcadeDrive(speed, rotation));
  }

  public CommandBase stop() {
    return runOnce(() -> diffDrive.stopMotor());
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
  public void periodic() {
    // This method will be called once per scheduler run
    //REVPhysicsSim.getInstance().run();
    odometry.update(Robot.isReal() ? ahrs.getRotation2d() : new Rotation2d(gyroSimAngle),
                    leftEncoder.getPosition(),
                    rightEncoder.getPosition());
    field.setRobotPose(odometry.getPoseMeters());

    SmartDashboard.putNumber("gyro angle", ahrs.getAngle());
    SmartDashboard.putNumber("pitch", ahrs.getPitch());
    SmartDashboard.putNumber("yaw", ahrs.getYaw());
    SmartDashboard.putNumber("roll", ahrs.getRoll());
    diffDrive.feed();
  }

  public void simulationInit() {
    REVPhysicsSim.getInstance().addSparkMax(frontLeft, DCMotor.getNEO(2));
    // REVPhysicsSim.getInstance().addSparkMax(rearLeft, DCMotor.getNEO(1));
    REVPhysicsSim.getInstance().addSparkMax(frontRight, DCMotor.getNEO(2));
    // REVPhysicsSim.getInstance().addSparkMax(rearRight, DCMotor.getNEO(1));
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    //REVPhysicsSim.getInstance().run();
    REVPhysicsSim.getInstance().run();
    diffSim.setInputs(frontLeft.get() * RobotController.getInputVoltage(),
                       frontRight.get() * RobotController.getInputVoltage());
    diffSim.update(Robot.kDefaultPeriod);
    leftSparkSim.getDouble("Position").set(diffSim.getLeftPositionMeters());
    leftSparkSim.getDouble("Velocity").set(diffSim.getLeftVelocityMetersPerSecond());
    rightSparkSim.getDouble("Position").set(diffSim.getRightPositionMeters());
    rightSparkSim.getDouble("Velocity").set(diffSim.getRightVelocityMetersPerSecond());
    gyroSimAngle += DriveConstants.kDriveKinematics.toChassisSpeeds(new DifferentialDriveWheelSpeeds(diffSim.getLeftVelocityMetersPerSecond(), diffSim.getRightVelocityMetersPerSecond())).omegaRadiansPerSecond
        * Robot.kDefaultPeriod;
  }
}
