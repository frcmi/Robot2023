// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kSpeedMultiplier = 0.75;
    public static final double kSpeedSlewRate = 0.5;
    public static final double kRotationMultiplier = 0.3;
    public static final double kElevatorSpeed = 0.5;
    public static final double kArmSpeed = 0.5;
    public static final double kArmDeadzone = 0.2;
  }

  public static class DriveConstants {
    public static final int currentLimit = 35;
    //Currently placeholder values, make sure to update before testing
    public static final int kFrontLeftMotorId = 1;
    public static final int kRearLeftMotorId = 3;
    public static final int kFrontRightMotorId = 2;
    public static final int kRearRightMotorId = 4;

    public static final double kWheelDiameter = Units.inchesToMeters(5.0);
    public static final double kWheelCircumference = kWheelDiameter * Math.PI;
    public static final double kWheelGearRatio = 8.89839; // Joey: dt should be 11:52 into 34:64
    public static final double kWheelEncoderDistancePerRotation = kWheelCircumference / kWheelGearRatio;
    public static final double kS = 0.20995;
    public static final double kV = 2.6071;
    public static final double kA = 1.127;
    public static final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV, kA);
    public static final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(0.5);
    public static final double kMaxSpeedMetersPerSecond = 4;
    public static final double kMaxAccelerationMetersPerSecondSquared = 2;
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
    public static final HashMap<String, Command> kEventMap = new HashMap<>();
    public static final PathConstraints kPathConstraints = new PathConstraints(4, 3);
  }

  public static class LEDConstants {
    public static final Color8Bit kYellow = new Color8Bit(255, 255, 0);
    public static final Color8Bit kPurple = new Color8Bit(255, 0, 255);
    public static final Color8Bit kInitialMaroon = new Color8Bit(144, 56, 32);
    public static final int kLightsPerFoot = 9;
    public static final int[] kLightPorts = {0, 1}; // <== Placeholder!!
    public static final int[] kLightsLengthsArray = {kLightsPerFoot, kLightsPerFoot};
  }

  public static class IntakeConstants {
    public static final int kNeoEncoderResolutionCPR = 42;
    public static final int kMotorId = 9;
    public static final double kIntakeSpeed = 0.5;
    public static final double kReleaseSpeed = 0.2 * -1;
    public static final double kReleaseTime = 1.0;
    public static final int kCurrentLimit = 15;
    // Amps drawn by manipulator showing current spike (to detect if grabbed object) 
    public static final double kCurrentThreshold = 15.0;
    // Time to keep running intake under current spike
    public static final double kIntakeTime = 0.75;
  }
  
  public static class ElevatorConstants {
    public static final int kNeoEncoderResolutionCPR = 42;
    public static final int kLeftMotorId = 5;
    public static final int kRightMotorId = 6;
    public static final double kElevatorGearRatio = 15.0;
    public static final int kCurrentLimit = 40;
    public static final double maxPos = 0.55;
    public static final double minPos = -0.01;
    public static final double raisedPos = 0.4;
    public static final double loweredPos = 0.0;
    // PID parameters
    public static final double kP = 9;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    // FeedForward parameters
    public static final double kS = 0.078775;
    public static final double kG = 0.1;
    public static final double kV = 11.961;
    public static final double kA = 0.47998;
    // Motion Profile
    public static final double kMaxVel = 1.5;
    public static final double kMaxAccel = 0.5;

    public static final double kSprocketDiameter = 0.048; //metres
    public static final double kSprocketCircumference = Math.PI * kSprocketDiameter;
    public static final double kElevatorEncoderDistancePerRotation = kSprocketCircumference / kElevatorGearRatio;
  }

  public static class ArmConstants {
    public static final int kNeoEncoderResolutionCPR = 42;
    public static final int kLeftMotorId = 7;
    public static final int kRightMotorId = 8;
    public static final double kArmGearRatio = 180.0/1.0;
    // 0 should be the middle
    public static final double encoderOffset = Math.toRadians(224.45);
    // Lowest safe rotation relative to encoderOffset 
    public static final double minAngle = -Math.toRadians(125);
    // Highest safe rotation relative to encoderOffset 
    public static final double maxAngle = Math.toRadians(160);
    public static final int kEncoderDIOPort = 0;
    public static final int kCurrentLimit = 40;
    // PID parameters
    public static final double kP = 12;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    // FeedForward parameters
    // regression from F = kG cos Theta is 0.265
    public static final double kS = 0.0;
    public static final double kG = 0.16;
    public static final double kV = 3.5;
    public static final double kA = 0.01;
    // Motion Profile
    public static final double kMaxVel = 6;
    public static final double kMaxAccel = 2;
  }
}
