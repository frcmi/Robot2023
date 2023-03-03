// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

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
    public static final double kSpeedMultiplier = 0.2;
    public static final double kRotationMultiplier = 0.2;
    public static final double kElevatorSpeed = 0.3;
    public static final double kArmSpeed = 0.3;
  }

  public static class DriveConstants {
    public static final double kMovementMultiplier = 0.5;

    //Currently placeholder values, make sure to update before testing
    public static final int kFrontLeftMotorId = 1;
    public static final int kRearLeftMotorId = 2;
    public static final int kFrontRightMotorId = 3;
    public static final int kRearRightMotorId = 4;

    public static final double kTrackwidthMeters = 0.62898; //Need to measure
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);

    public static final int kEncoderCPR = 2048;
    public static final double kWheelDiameterMeters = 0.1524; // 6in wheel diameter
    public static final double kGearRatio = (double) 111/11; //Change I think
    public static final double kEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (kWheelDiameterMeters * Math.PI) / (kEncoderCPR * kGearRatio);

    public static final double ksVolts = 0.25355;
    public static final double kvVoltSecondsPerMeter = 2.0519;
    public static final double kaVoltSecondsSquaredPerMeter = 0.53639;

    // Example value only - as above, this must be tuned for your drive!
    public static final double kPDriveVel = 3.7728;
  }

  public static class IntakeConstants {
    public static final int kMotorId = 9;
    public static final double kIntakeSpeed = 0.3;
    public static final double kReleaseSpeed = 0.2 * -1;
    public static final double kReleaseTime = 1.0;
    public static final int kCurrentLimit = 15;
    // Amps drawn by manipulator showing current spike (to detect if grabbed object) 
    public static final double kCurrentThreshold = 15.0;
    // Time to keep running intake under current spike
    public static final double kIntakeTime = 0.75;
  }
  
  public static class ElevatorConstants {
    public static final int kLeftMotorId = 5;
    public static final int kRightMotorId = 6;
    public static final double kGearRatio = 1.0/15.0;
    public static final int kCurrentLimit = 40;
    // PID parameters
    public static final double kP = 0.3;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    // FeedForward parameters
    public static final double kS = 0.0;
    public static final double kG = 0.0;
    public static final double kV = 0.0;
    public static final double kA = 0.0;
    // Motion Profile
    public static final double kMaxVel = 0.5;
    public static final double kMaxAccel = 0.1;
  }

  public static class ArmConstants {
    public static final int kLeftMotorId = 7;
    public static final int kRightMotorId = 8;
    public static final double kArmGearRatio = 1.0/180.0;
    public static final int kCurrentLimit = 40;
    // PID parameters
    public static final double kP = 0.3;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    // FeedForward parameters
    public static final double kS = 0.0;
    public static final double kG = 0.0;
    public static final double kV = 0.0;
    public static final double kA = 0.0;
    // Motion Profile
    public static final double kMaxVel = 0.5;
    public static final double kMaxAccel = 0.1;
  }
}
